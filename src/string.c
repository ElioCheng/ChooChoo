#include "string.h"

int strcmp(const char *str1, const char *str2)
{
	while (*str1 && *str2 && *str1 == *str2) {
		str1++;
		str2++;
	}
	return *str1 - *str2;
}

void *memcpy(void *dest, const void *src, size_t n)
{
	char *d = dest;
	const char *s = src;
	while (n--) {
		*d++ = *s++;
	}
	return dest;
}

void strcpy(char *dest, const char *src)
{
	while (*src != '\0') {
		*dest++ = *src++;
	}
	*dest = '\0';
}

void strncpy(char *dest, const char *src, size_t n)
{
	while (n--) {
		*dest++ = *src++;
	}
}

int strlen(const char *str)
{
	int len = 0;
	while (*str++) {
		len++;
	}
	return len;
}

int strncmp(const char *s1, const char *s2, size_t n)
{
	while (n--) {
		if (*s1 != *s2) {
			return *s1 - *s2;
		}
		s1++;
		s2++;
	}
	return 0;
}

void strcat(char *dest, const char *src)
{
	while (*dest) {
		dest++;
	}
	while (*src) {
		*dest++ = *src++;
	}
	*dest = '\0';
}

void strncat(char *dest, const char *src, size_t n)
{
	while (*dest) {
		dest++;
	}
	while (n--) {
		*dest++ = *src++;
	}
	*dest = '\0';
}

void *memset(void *s, int c, size_t n)
{
	for (char *it = (char *)s; n > 0; --n)
		*it++ = c;
	return s;
}

void *memmove(void *dest, const void *src, size_t n)
{
	char *d = dest;
	const char *s = src;
	if (d < s) {
		while (n--)
			*d++ = *s++;
	} else {
		d += n;
		s += n;
		while (n--)
			*--d = *--s;
	}
	return dest;
}

char *strchr(const char *str, char c)
{
	while (*str) {
		if (*str == c) {
			return (char *)str;
		}
		str++;
	}
	return NULL;
}

int memcmp(const void *s1, const void *s2, size_t n)
{
	const char *p1 = s1;
	const char *p2 = s2;
	while (n--) {
		if (*p1 != *p2) {
			return *p1 - *p2;
		}
		p1++;
		p2++;
	}
	return 0;
}