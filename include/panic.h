#ifndef PANIC_H
#define PANIC_H

#include "klog.h"

#define BUG_ON(x) do { if (x) panic("BUG_ON: %s", #x); } while (0)

#define panic(fmt, ...) do_panic(__func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__)
void do_panic(const char *func, const char *line, const char *fmt, ...);

#endif
