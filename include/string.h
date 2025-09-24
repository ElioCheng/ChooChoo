#ifndef __STRING_H__
#define __STRING_H__

#include "types.h"

int strcmp(const char *str1, const char *str2);
void *memcpy(void *dest, const void *src, size_t n);
void strcpy(char *dest, const char *src);
void strncpy(char *dest, const char *src, size_t n);
int strlen(const char *str);
int strncmp(const char *s1, const char *s2, size_t n);
void strcat(char *dest, const char *src);
void strncat(char *dest, const char *src, size_t n);
char *strchr(const char *str, char c);
int memcmp(const void *s1, const void *s2, size_t n);

void *memset(void *s, int c, size_t n);
void *memmove(void *dest, const void *src, size_t n);

#endif
