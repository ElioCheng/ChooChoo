#include "random.h"

static u64 random_seed = KERNEL_RANDOM_SEED;

u64 random(void) {
	// LCG: next = (a * seed + c) % m
	// https://en.wikipedia.org/wiki/Linear_congruential_generator
	random_seed = (random_seed * 1103515245UL + 12345UL) & 0x7FFFFFFF;
	return random_seed;
}

u64 random_range(u64 min, u64 max) {
	return random() % (max - min + 1) + min;
}
