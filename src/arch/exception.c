#include "arch/registers.h"
#include "klog.h"
#include "types.h"
#include "uapi/syscall.h"

extern void setup_exception_vector_table(void);

void exception_init(void) {
    setup_exception_vector_table();
    klog_info("Exception vector table initialized");
}
