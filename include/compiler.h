#ifndef COMPILER_H
#define COMPILER_H

#define UNUSED(x) (void)(x)

#define __maybe_unused __attribute__((unused))

#define __noreturn __attribute__((noreturn))

#define UNREACHABLE() __builtin_unreachable()

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define isb() __asm__ volatile("isb" ::: "memory")
#define dsb() __asm__ volatile("dsb sy" ::: "memory")

#endif /* compiler.h */
