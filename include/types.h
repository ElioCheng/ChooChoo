#ifndef TYPES_H
#define TYPES_H

typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef signed long long i64;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

typedef signed long ptr_t;
typedef unsigned long size_t;

#include <stdbool.h>

#define NULL ((void *)0)

#define ptr_to_ulong(p) ((unsigned long)(p))

#define INT_MAX (~0U >> 1)
#define UINT_MAX (~0U)

#endif /* type.h */
