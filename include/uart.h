#ifndef UART_H
#define UART_H

#include "sched.h"
#include <stddef.h>
#include "types.h"

// UART line definitions
#define CONSOLE 1
#define MARKLIN 2

// UART functions
void uart_init_buffers(size_t line);
void uart_config_and_enable(size_t line);
unsigned char uart_getc(size_t line);
void uart_putc(size_t line, char c);
void uart_putc_direct(size_t line, char c);
void uart_puts(size_t line, const char *buf);
void uart_putl(size_t line, const char *buf, size_t size);
void uart_printf(size_t line, size_t buf_size, const char *fmt, ...);
void uart_process_tx_buffers(void);
void uart_process_tx_buffers_blocking(void);
void uart_clear_buffer(size_t line);
void uart_buffer_status_print(void);
void uart_clear_pending_input(size_t line);
u8 uart_rx_has_data(size_t line);

// Interrupt control functions to support the IO server
void uart_enable_rx_interrupt(size_t line);
void uart_disable_rx_interrupt(size_t line);
void uart_enable_tx_interrupt(size_t line);
void uart_disable_tx_interrupt(size_t line);
void uart_clear_rx_interrupt(size_t line);
void uart_clear_tx_interrupt(size_t line);
void uart_clear_ms_interrupt(size_t line);
void uart_disable_ms_interrupt(size_t line);
u32 uart_get_interrupt_status(size_t line);
u8 uart_tx_has_space(size_t line);

void uart_init_interrupts(void);

#endif /* uart.h */
