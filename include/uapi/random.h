#ifndef __UAPI_RANDOM_H__
#define __UAPI_RANDOM_H__

#include "types.h"

#ifndef RANDOM_SEED
#define KERNEL_RANDOM_SEED 0x123456789ABCDEFULL
#endif

u64 random(void);
u64 random_range(u64 min, u64 max);

#endif