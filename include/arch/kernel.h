#ifndef KERNEL_H
#define KERNEL_H

#ifdef __aarch64__
#define ARCH_NAME "aarch64"
#else
#error "Unsupported architecture"
#endif

#endif /* KERNEL_H */