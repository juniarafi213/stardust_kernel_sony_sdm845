// SPDX-License-Identifier: GPL-2.0
/*
 * kernel/sched/vorpal_shaping.c
 *
 * Lapisan shaping tambahan di atas schedutil (bukan governor terpisah).
 * Dipanggil dari sugov_update_single() SETELAH get_next_freq() asli,
 * jadi semua util tracking (WALT), iowait boost, dan RT/DL handling bawaan
 * schedutil TETAP JALAN APA ADANYA — file ini cuma "membentuk ulang" angka
 * next_f yang sudah dihitung, sebelum benar-benar dikirim ke hardware.
 *
 * Cara pasang ke cpufreq_schedutil.c (cuma 2 baris):
 *
 *   1. Di bagian atas file, tambahkan:
 *        extern unsigned int rfx_apply_vorpal_shaping(int cpu,
 *                unsigned int freq, unsigned long max_cap, u64 time);
 *
 *   2. Di sugov_update_single(), tepat setelah baris:
 *        next_f = get_next_freq(sg_policy, util, max);
 *      tambahkan:
 *        next_f = rfx_apply_vorpal_shaping(sg_cpu->cpu, next_f, max, time);
 *
 * Tambahkan file ini ke kernel/sched/Makefile:
 *   obj-y += vorpal_shaping.o
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/atomic.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/cpufreq.h>
#include <linux/ktime.h>
#include <linux/input.h>
#include <linux/timekeeping.h>

/* ===================================================================== */
/* Tunable — masih placeholder, WAJIB di-tuning ulang lewat testing      */
/* ===================================================================== */

#define VORPAL_LITTLE_CAP_THRESHOLD 512   /* di bawah ini dianggap cluster Little */

#define VORPAL_G_BIG_FLOOR_PCT   85
#define VORPAL_G_BIG_CAP_PCT     100

#define VORPAL_G_LITTLE_CAP_PCT         100
#define VORPAL_G_LITTLE_FLOOR_PCT       58
#define VORPAL_G_LITTLE_FLOOR_ENTER_PCT 12
#define VORPAL_G_LITTLE_HOLD_NS (80 * NSEC_PER_MSEC)

#define VORPAL_D_LITTLE_CAP_PCT       82
#define VORPAL_D_LITTLE_BOOST_CAP_PCT 90
#define VORPAL_D_LITTLE_UI_FLOOR_PCT  58
#define VORPAL_D_BIG_UI_FLOOR_PCT     55
#define VORPAL_D_RAMP_DELTA_PCT       8
#define VORPAL_D_UI_BOOST_NS (200 * NSEC_PER_MSEC)
#define VORPAL_INPUT_WINDOW_NS (220 * NSEC_PER_MSEC)

#define VORPAL_THERMAL_STEP_NS      (6 * NSEC_PER_MSEC)
#define VORPAL_THERMAL_STEP_DOWN_PCT 2
#define VORPAL_THERMAL_STEP_UP_PCT   1
#define VORPAL_THERMAL_MIN_CAP_PCT   70

#define VORPAL_TEMP_GREEN_MC  48000
#define VORPAL_TEMP_YELLOW_MC 52000
#define VORPAL_TEMP_RED_MC    56000

/* ===================================================================== */
/* State global (gaming mode toggle, input timestamp, suhu)              */
/* ===================================================================== */

static atomic_t vorpal_gaming = ATOMIC_INIT(0);
static atomic64_t vorpal_input_ts_ns = ATOMIC64_INIT(0);

/*
 * Suhu (milli-Celsius). Diisi userspace lewat sysfs (thermal-engine/thermald
 * kamu sendiri yang tulis ke sini secara berkala), karena binding langsung ke
 * satu thermal_zone tertentu berisiko salah sensor antar-device. Kalau mau
 * baca otomatis dari kernel thermal framework, itu langkah lanjutan terpisah.
 */
static atomic_t vorpal_temp_mc = ATOMIC_INIT(0);

/* ===================================================================== */
/* State per-CPU (indexed manual, tidak nempel ke struct sugov_cpu)      */
/* ===================================================================== */

struct vorpal_cpu_state {
	unsigned int prev_upct;
	u64 ui_boost_end_ns;
	u64 little_busy_hold_ns;
	int thermal_applied_pct;
	u64 thermal_step_ns;
};

static DEFINE_PER_CPU(struct vorpal_cpu_state, vorpal_cpu_state);

/* ===================================================================== */
/* Helper */
/* ===================================================================== */

static inline bool vorpal_is_little(unsigned long max_cap)
{
	return max_cap <= (unsigned long)VORPAL_LITTLE_CAP_THRESHOLD;
}

static inline unsigned int vorpal_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

static inline bool vorpal_input_active(u64 time)
{
	u64 ts = (u64)atomic64_read(&vorpal_input_ts_ns);
	return ts && (time - ts) < VORPAL_INPUT_WINDOW_NS;
}

static int vorpal_temp_to_cap(int t_mc)
{
	if (t_mc < VORPAL_TEMP_GREEN_MC)
		return 100;
	if (t_mc < VORPAL_TEMP_YELLOW_MC)
		return 100 - (t_mc - VORPAL_TEMP_GREEN_MC) * 2 / 1000;
	if (t_mc < VORPAL_TEMP_RED_MC)
		return 90 - (t_mc - VORPAL_TEMP_YELLOW_MC) * 3 / 1000;
	return 75;
}

static unsigned int vorpal_thermal_clamp(struct vorpal_cpu_state *st,
					  unsigned int freq, unsigned int fmax,
					  u64 time)
{
	int target = vorpal_temp_to_cap(atomic_read(&vorpal_temp_mc));
	int applied = st->thermal_applied_pct ? st->thermal_applied_pct : 100;

	if ((s64)(time - st->thermal_step_ns) >= (s64)VORPAL_THERMAL_STEP_NS) {
		if (applied > target)
			applied -= VORPAL_THERMAL_STEP_DOWN_PCT;
		else if (applied < target)
			applied += VORPAL_THERMAL_STEP_UP_PCT;
		applied = clamp(applied, VORPAL_THERMAL_MIN_CAP_PCT, 100);
		st->thermal_applied_pct = applied;
		st->thermal_step_ns = time;
	}

	if (applied < 100) {
		unsigned int cap = vorpal_pct(fmax, applied);
		if (freq > cap)
			freq = cap;
	}
	return freq;
}

/* ===================================================================== */
/* Fungsi utama — dipanggil dari sugov_update_single()                   */
/* ===================================================================== */

/**
 * rfx_apply_vorpal_shaping - bentuk ulang next_f hasil get_next_freq() asli
 * @cpu:     nomor CPU yang lagi diupdate (dari sg_cpu->cpu)
 * @freq:    next_f hasil get_next_freq() schedutil asli — INI YANG DIBENTUK ULANG
 * @max_cap: kapasitas arch CPU ini (buat bedain cluster Big/Little)
 * @time:    timestamp saat ini (ns), sama seperti yang dipakai schedutil
 *
 * Fungsi ini TIDAK menghitung util dari nol — dia cuma menerapkan floor/cap
 * gaming, UI-boost harian, dan thermal step clamp DI ATAS keputusan schedutil
 * yang sudah ada. Kalau freq hasil schedutil sudah lebih tinggi dari floor,
 * floor tidak berefek (max() semantics) — jadi tidak pernah "menurunkan"
 * keputusan schedutil, cuma menaikkan floor atau menurunkan cap/thermal.
 */
unsigned int rfx_apply_vorpal_shaping(int cpu, unsigned int freq,
				       unsigned long max_cap, u64 time)
{
	struct vorpal_cpu_state *st = &per_cpu(vorpal_cpu_state, cpu);
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	unsigned int fmax, fmin, upct;
	bool little, gaming;

	if (!policy)
		return freq;

	fmax = policy->cpuinfo.max_freq;
	fmin = policy->cpuinfo.min_freq;
	if (!fmax)
		return freq;

	little = vorpal_is_little(max_cap);
	gaming = atomic_read(&vorpal_gaming) != 0;
	upct = (unsigned int)((u64)freq * 100 / fmax);

	if (gaming) {
		if (!little) {
			unsigned int fl = vorpal_pct(fmax, VORPAL_G_BIG_FLOOR_PCT);
			unsigned int cap = vorpal_pct(fmax, VORPAL_G_BIG_CAP_PCT);

			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		} else {
			unsigned int cap = vorpal_pct(fmax, VORPAL_G_LITTLE_CAP_PCT);
			unsigned int fl = 0;
			bool busy;

			if (upct > VORPAL_G_LITTLE_FLOOR_ENTER_PCT)
				st->little_busy_hold_ns = time + VORPAL_G_LITTLE_HOLD_NS;

			busy = (upct > VORPAL_G_LITTLE_FLOOR_ENTER_PCT) ||
			       (st->little_busy_hold_ns && time < st->little_busy_hold_ns);

			if (busy)
				fl = vorpal_pct(fmax, VORPAL_G_LITTLE_FLOOR_PCT);

			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		}
	} else {
		bool ui_active;

		if (upct > st->prev_upct && upct - st->prev_upct >= VORPAL_D_RAMP_DELTA_PCT)
			st->ui_boost_end_ns = time + VORPAL_D_UI_BOOST_NS;
		st->prev_upct = upct;

		ui_active = vorpal_input_active(time) ||
			    (st->ui_boost_end_ns && time < st->ui_boost_end_ns);

		if (little) {
			unsigned int cap = ui_active ?
				vorpal_pct(fmax, VORPAL_D_LITTLE_BOOST_CAP_PCT) :
				vorpal_pct(fmax, VORPAL_D_LITTLE_CAP_PCT);
			if (freq > cap)
				freq = cap;
			if (ui_active) {
				unsigned int fl = vorpal_pct(fmax, VORPAL_D_LITTLE_UI_FLOOR_PCT);
				if (freq < fl)
					freq = fl;
			}
		} else if (ui_active) {
			unsigned int fl = vorpal_pct(fmax, VORPAL_D_BIG_UI_FLOOR_PCT);
			if (freq < fl)
				freq = fl;
		}
	}

	freq = vorpal_thermal_clamp(st, freq, fmax, time);
	freq = clamp(freq, fmin, fmax);

	return freq;
}
EXPORT_SYMBOL_GPL(rfx_apply_vorpal_shaping);

/* ===================================================================== */
/* Sysfs — /sys/kernel/vorpal/{gaming_mode,temp_mc}                      */
/* ===================================================================== */

static struct kobject *vorpal_kobj;

static ssize_t gaming_mode_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&vorpal_gaming));
}

static ssize_t gaming_mode_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&vorpal_gaming, !!val);
	return count;
}
static struct kobj_attribute gaming_mode_attr =
	__ATTR(gaming_mode, 0644, gaming_mode_show, gaming_mode_store);

static ssize_t temp_mc_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&vorpal_temp_mc));
}

static ssize_t temp_mc_store(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&vorpal_temp_mc, val);
	return count;
}
static struct kobj_attribute temp_mc_attr =
	__ATTR(temp_mc, 0644, temp_mc_show, temp_mc_store);

static struct attribute *vorpal_attrs[] = {
	&gaming_mode_attr.attr,
	&temp_mc_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(vorpal);

static int __init vorpal_shaping_init(void)
{
	vorpal_kobj = kobject_create_and_add("vorpal", kernel_kobj);
	if (!vorpal_kobj)
		return -ENOMEM;

	if (sysfs_create_groups(vorpal_kobj, vorpal_groups)) {
		kobject_put(vorpal_kobj);
		return -ENOMEM;
	}

	pr_info("vorpal_shaping: initialized (gaming_mode=%d)\n",
		atomic_read(&vorpal_gaming));
	return 0;
}
late_initcall(vorpal_shaping_init);

/*
 * ============================================================
 * MASIH BELUM ADA (langkah lanjutan setelah ini jalan):
 * - Frame-time feed dari userspace (fas-rs/frame_analyzer) buat proactive
 *   frame boost, seperti rfx_frame_account() di source Vorpal asli.
 *   Tambahkan: /sys/kernel/vorpal/frame_time_us (write-only dari userspace),
 *   simpan di atomic, cek di rfx_apply_vorpal_shaping() apakah perlu boost.
 * - Auto-baca suhu dari kernel thermal framework (opsional, sekarang manual
 *   via temp_mc sysfs — perlu daemon userspace yang polling & menulis ke sana)
 * - Tuning ulang SEMUA angka *_PCT di atas lewat testing nyata di device kamu
 * ============================================================
 */
