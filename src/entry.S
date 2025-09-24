.section ".text"

.global sync_el0_handler
.global irq_el0_handler
.global other_handler

irq_el1_handler_sp0:
serror_el1_handler_spx:
sync_el1_handler_sp0:
fiq_el1_handler_sp0:
serror_el1_handler_sp0:
sync_el1_handler_spx:
irq_el1_handler_spx:
fiq_el1_handler_spx:
    bl other_handler
    b .

fiq_el0_handler:
serror_el0_handler:
sync_el0_handler_32:
irq_el0_handler_32:
fiq_el0_handler_32:
serror_el0_handler_32:
    bl other_handler
    b .


// Store EL0 context to a context_t structure on the stack
.macro store_el0_context, offset
    // Store general purpose registers x0-x30
    stp x0, x1, [sp, #(\offset + 8 * 0)]
    stp x2, x3, [sp, #(\offset + 8 * 2)]
    stp x4, x5, [sp, #(\offset + 8 * 4)]
    stp x6, x7, [sp, #(\offset + 8 * 6)]
    stp x8, x9, [sp, #(\offset + 8 * 8)]
    stp x10, x11, [sp, #(\offset + 8 * 10)]
    stp x12, x13, [sp, #(\offset + 8 * 12)]
    stp x14, x15, [sp, #(\offset + 8 * 14)]
    stp x16, x17, [sp, #(\offset + 8 * 16)]
    stp x18, x19, [sp, #(\offset + 8 * 18)]
    stp x20, x21, [sp, #(\offset + 8 * 20)]
    stp x22, x23, [sp, #(\offset + 8 * 22)]
    stp x24, x25, [sp, #(\offset + 8 * 24)]
    stp x26, x27, [sp, #(\offset + 8 * 26)]
    stp x28, x29, [sp, #(\offset + 8 * 28)]
    str x30, [sp, #(\offset + 8 * 30)]

    // Store stack pointer from EL0
    mrs x19, sp_el0
    str x19, [sp, #(\offset + 8 * 32)]

    // Store program counter (ELR_EL1)
    mrs x19, elr_el1
    str x19, [sp, #(\offset + 8 * 33)]

    // Store processor state (SPSR_EL1)
    mrs x19, spsr_el1
    str x19, [sp, #(\offset + 8 * 34)]

    // Store exception link register (same as PC for EL0)
    mrs x19, elr_el1
    str x19, [sp, #(\offset + 8 * 35)]

    // Store TPIDR_EL0 (thread pointer)
    mrs x19, tpidr_el0
    str x19, [sp, #(\offset + 8 * 36)]
.endm

// Restore EL0 context from a context_t structure on the stack
.macro restore_el0_context, offset, el0_irq_enabled
    // Restore TPIDR_EL0 (thread pointer)
    ldr x19, [sp, #(\offset + 8 * 36)]
    msr tpidr_el0, x19

    // Restore processor state (SPSR_EL1)
    ldr x19, [sp, #(\offset + 8 * 34)]

    .if \el0_irq_enabled
        bic x19, x19, #(1 << 7)
    .endif
    msr spsr_el1, x19

    // Restore program counter (ELR_EL1)
    ldr x19, [sp, #(\offset + 8 * 33)]
    msr elr_el1, x19

    // Restore stack pointer to EL0
    ldr x19, [sp, #(\offset + 8 * 32)]
    msr sp_el0, x19

    // Restore general purpose registers x0-x30
    ldr x30, [sp, #(\offset + 8 * 30)]
    ldp x28, x29, [sp, #(\offset + 8 * 28)]
    ldp x26, x27, [sp, #(\offset + 8 * 26)]
    ldp x24, x25, [sp, #(\offset + 8 * 24)]
    ldp x22, x23, [sp, #(\offset + 8 * 22)]
    ldp x20, x21, [sp, #(\offset + 8 * 20)]
    ldp x18, x19, [sp, #(\offset + 8 * 18)]
    ldp x16, x17, [sp, #(\offset + 8 * 16)]
    ldp x14, x15, [sp, #(\offset + 8 * 14)]
    ldp x12, x13, [sp, #(\offset + 8 * 12)]
    ldp x10, x11, [sp, #(\offset + 8 * 10)]
    ldp x8, x9, [sp, #(\offset + 8 * 8)]
    ldp x6, x7, [sp, #(\offset + 8 * 6)]
    ldp x4, x5, [sp, #(\offset + 8 * 4)]
    ldp x2, x3, [sp, #(\offset + 8 * 2)]
    ldp x0, x1, [sp, #(\offset + 8 * 0)]
.endm

.macro el0_entry, label
    .align 7
    // Allocate space for context_t structure on stack
    // context_t contains struct arch_regs which is:
    // x[32] = 32 * 8 = 256 bytes
    // sp, pc, spsr, elr, tpidr = 5 * 8 = 40 bytes
    // Total = 296 bytes, round up to 16-byte boundary = 304 bytes
    sub sp, sp, #304

    // Store the context starting at sp
    store_el0_context 0

    // Align stack to 16-byte boundary for function call
    mov x0, sp
    and x0, x0, #-16
    mov sp, x0

    mov x0, sp
    bl \label

    // a task should be scheduled and we should not reach here
.endm

// Function to switch to user mode with new task context
// args: x0 = pointer to context_t structure on stack to restore

.global switch_to_user_mode
switch_to_user_mode:
    // x0 = pointer to context_t structure on stack

    sub sp, sp, #304

    mov x1, x0
    mov x0, sp
    mov x2, #304
    bl memcpy

    restore_el0_context 0, 1

    // add sp, sp, #304
    mov sp, #0x80000
    eret

.global setup_exception_vector_table
setup_exception_vector_table:
    adr x0, exception_vector_table
    msr vbar_el1, x0
    ret

.global exception_vector_table
.align 11
exception_vector_table:
    // Current EL with SP0
    b   sync_el1_handler_sp0    // Synchronous
    .align 7                    // Align to 128 bytes
    b   irq_el1_handler_sp0     // IRQ
    .align 7                    // Align to 128 bytes
    b   fiq_el1_handler_sp0     // FIQ
    .align 7                    // Align to 128 bytes
    b   serror_el1_handler_sp0  // SError
    .align 7                    // Align to 128 bytes

    // Current EL with SPx
    b   sync_el1_handler_spx    // Synchronous
    .align 7                    // Align to 128 bytes
    b   irq_el1_handler_spx     // IRQ
    .align 7                    // Align to 128 bytes
    b   fiq_el1_handler_spx     // FIQ
    .align 7                    // Align to 128 bytes
    b   serror_el1_handler_spx  // SError
    .align 7                    // Align to 128 bytes

    // Lower EL using AArch64
    el0_entry sync_el0_handler  // Synchronous
    .align 7                    // Align to 128 bytes
    el0_entry irq_el0_handler   // IRQ
    .align 7                    // Align to 128 bytes
    b   fiq_el0_handler         // FIQ
    .align 7                    // Align to 128 bytes
    b   serror_el0_handler      // SError
    .align 7                    // Align to 128 bytes

    // Lower EL using AArch32
    b   sync_el0_handler_32     // Synchronous
    .align 7                    // Align to 128 bytes
    b   irq_el0_handler_32      // IRQ
    .align 7                    // Align to 128 bytes
    b   fiq_el0_handler_32      // FIQ
    .align 7                    // Align to 128 bytes
    b   serror_el0_handler_32   // SError
