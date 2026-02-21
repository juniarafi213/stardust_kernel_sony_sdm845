#ifndef _ASM_ARM64_ARCH_HWEIGHT_H_
#define _ASM_ARM64_ARCH_HWEIGHT_H_

#include <asm/types.h>

static __always_inline unsigned int __arch_hweight32(unsigned int w)
{
	unsigned int res;
	w -= (w >> 1) & 0x55555555;
	w =  (w & 0x33333333) + ((w >> 2) & 0x33333333);
	w =  (w + (w >> 4)) & 0x0f0f0f0f;
	res = (w * 0x01010101) >> 24;
	return res;
}

static __always_inline unsigned int __arch_hweight16(unsigned int w)
{
	return __arch_hweight32(w & 0xffff);
}

static __always_inline unsigned int __arch_hweight8(unsigned int w)
{
	return __arch_hweight32(w & 0xff);
}

static __always_inline unsigned long __arch_hweight64(__u64 w)
{
	unsigned long res;
	w -= (w >> 1) & 0x5555555555555555ul;
	w =  (w & 0x3333333333333333ul) + ((w >> 2) & 0x3333333333333333ul);
	w =  (w + (w >> 4)) & 0x0f0f0f0f0f0f0f0ful;
	res = (w * 0x0101010101010101ul) >> 56;
	return res;
}

#endif
