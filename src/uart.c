#include "uart.h"
#include "arch/rpi.h"
#include "klog.h"
#include "printf.h"
#include "panic.h"
#include "timer/time.h"
#include "types.h"
#include "sched.h"
#include "interrupt.h"
#include "arch/interrupts.h"
#include "event.h"
#include "compiler.h"
#include <stdarg.h>

#ifndef MMIO_BASE
#define MMIO_BASE (char *)0xFE000000
#endif

// Define a macro to log to memory when the UART is full
#define LOG_TO_MEMORY(msg, ...)                          \
	do {                                             \
		u32 dest = klog_get_destinations();      \
		klog_set_destinations(KLOG_DEST_MEMORY); \
		klog_warning(msg, ##__VA_ARGS__);        \
		klog_set_destinations(dest);             \
	} while (0)

/*********** UART CONTROL ************************ ************/

static char *const UART0_BASE = (char *)(MMIO_BASE + 0x201000);
static char *const UART3_BASE = (char *)(MMIO_BASE + 0x201600);
static char *const GPIO_BASE = (char *)(MMIO_BASE + 0x200000);

static char *const line_uarts[] = { NULL, UART0_BASE, UART3_BASE };
#define UART_REG(line, offset) (*(volatile u32 *)(line_uarts[line] + offset))

// UART register offsets
static const u32 UART_DR = 0x00;
static const u32 UART_FR = 0x18;
static const u32 UART_IBRD = 0x24;
static const u32 UART_FBRD = 0x28;
static const u32 UART_LCRH = 0x2c;
static const u32 UART_CR = 0x30;

// masks for specific fields in the UART registers
static const u32 UART_FR_RXFE = 0x10;
static const u32 UART_FR_TXFF = 0x20;
static const u32 UART_FR_RXFF = 0x40;
static const u32 UART_FR_TXFE = 0x80;

static const u32 UART_CR_UARTEN = 0x01;
static const u32 UART_CR_LBE = 0x80;
static const u32 UART_CR_TXE = 0x100;
static const u32 UART_CR_RXE = 0x200;
static const u32 UART_CR_RTS = 0x800;
static const u32 UART_CR_RTSEN = 0x4000;
static const u32 UART_CR_CTSEN = 0x8000;

static const u32 UART_LCRH_PEN = 0x2;
static const u32 UART_LCRH_EPS = 0x4;
static const u32 UART_LCRH_STP2 = 0x8;
static const u32 UART_LCRH_FEN = 0x10;
static const u32 UART_LCRH_WLEN_LOW = 0x20;
static const u32 UART_LCRH_WLEN_HIGH = 0x40;

// UART interrupt register offsets
static const u32 UART_IMSC = 0x38; // Interrupt Mask Set Clear register
static const u32 UART_ICR = 0x44; // Interrupt Clear register
static const u32 UART_MIS = 0x40; // Masked Interrupt Status register

// UART interrupt bits
static const u32 UART_INT_RX = 0x10; // Receive interrupt
static const u32 UART_INT_TX = 0x20; // Transmit interrupt
static const u32 UART_INT_RT = 0x40; // Receive timeout interrupt
static const u32 UART_INT_MS = 0x01; // Modem status interrupt (CTS changes)
static const u32 UART_INT_ERR = 0x780; // Error interrupts (FE, PE, BE, OE)

// Console buffer only
#define UART_TX_BUFFER_SIZE 102400 // 100kB
typedef struct {
	char buffer[UART_TX_BUFFER_SIZE];
	size_t head;
	size_t tail;
	size_t count;
} uart_tx_buffer_t;

static uart_tx_buffer_t console_tx_buffer = { 0 };

// Initialize UART buffers
void uart_init_buffers(size_t line)
{
	if (line == CONSOLE) {
		console_tx_buffer.count = 0;
		console_tx_buffer.head = 0;
		console_tx_buffer.tail = 0;
	}
}

void uart_clear_buffer(size_t line)
{
	if (line == CONSOLE) {
		console_tx_buffer.count = 0;
		console_tx_buffer.head = 0;
		console_tx_buffer.tail = 0;
	}
}

void uart_clear_pending_input(size_t line)
{
	while (!(UART_REG(line, UART_FR) & UART_FR_RXFE)) {
		(void)UART_REG(line, UART_DR); // Read and discard any pending input
	}
}

void uart_process_tx_buffers(void)
{
	u64 max_retries = 1000;

	for (u64 i = 0; i < max_retries; i++) {
		// Check if FIFO is not full (console has FIFOs enabled)
		bool has_space = !(UART_REG(CONSOLE, UART_FR) & UART_FR_TXFF);

		while (console_tx_buffer.count > 0 && has_space) {
			UART_REG(CONSOLE, UART_DR) = console_tx_buffer.buffer[console_tx_buffer.tail];
			console_tx_buffer.tail = (console_tx_buffer.tail + 1) % UART_TX_BUFFER_SIZE;
			console_tx_buffer.count--;

			// Recheck space for next iteration
			has_space = !(UART_REG(CONSOLE, UART_FR) & UART_FR_TXFF);
		}

		// Wait a bit to let go of the FIFO so that we can squeeze in a few more bytes
		for (u64 j = 0; j < 1000; j++) {
			asm volatile("nop");
		}
	}
}

void uart_buffer_status_print()
{
	klog_debug("UART console: Buffer status, count: %d, head: %d, tail: %d", console_tx_buffer.count,
		   console_tx_buffer.head, console_tx_buffer.tail);
}

void uart_process_tx_buffers_blocking(void)
{
	while (console_tx_buffer.count > 0) {
		bool has_space = !(UART_REG(CONSOLE, UART_FR) & UART_FR_TXFF);
		if (has_space) {
			UART_REG(CONSOLE, UART_DR) = console_tx_buffer.buffer[console_tx_buffer.tail];
			console_tx_buffer.tail = (console_tx_buffer.tail + 1) % UART_TX_BUFFER_SIZE;
			console_tx_buffer.count--;
		} else {
			continue;
		}
	}
}

void uart_putc(size_t line, char c)
{
	if (line != CONSOLE) {
		return;
	}

	if (console_tx_buffer.count >= UART_TX_BUFFER_SIZE) {
		uart_process_tx_buffers();
		if (console_tx_buffer.count >= UART_TX_BUFFER_SIZE) {
			panic("UART console: Transmit buffer full, dropping byte");
			LOG_TO_MEMORY("UART console: Transmit buffer full, dropping byte");
			return;
		}
	}

	// Add byte to buffer
	console_tx_buffer.buffer[console_tx_buffer.head] = c;
	console_tx_buffer.head = (console_tx_buffer.head + 1) % UART_TX_BUFFER_SIZE;
	console_tx_buffer.count++;
}

void uart_putc_direct(size_t line, char c)
{
	if (line == CONSOLE) {
		// Check if FIFO is full
		if (UART_REG(line, UART_FR) & UART_FR_TXFF) {
			LOG_TO_MEMORY("UART console: Transmit FIFO full");
			return;
		}
		UART_REG(line, UART_DR) = c;
	}
}

// Configure the line properties (e.g, parity, baud rate) of a UART and ensure that it is enabled
void uart_config_and_enable(size_t line)
{
	u32 baud_ival, baud_fval;
	u32 stop2;
	u32 fifo_enable;

	switch (line) {
	// setting baudrate to approx. 115246.09844 (best we can do); 1 stop bit
	case CONSOLE:
		baud_ival = 26;
		baud_fval = 2;
		stop2 = 0;
		fifo_enable = UART_LCRH_FEN; // Enable FIFOs for console
		break;
	// setting baudrate to 2400; 2 stop bits
	case MARKLIN:
		baud_ival = 1250;
		baud_fval = 0;
		stop2 = UART_LCRH_STP2;
		fifo_enable = UART_LCRH_FEN; // Enable FIFOs for Marklin
		break;
	default:
		return;
	}

	uart_init_buffers(line);

	// line control registers should not be changed while the UART is enabled, so disable it
	u32 cr_state = UART_REG(line, UART_CR);

	UART_REG(line, UART_CR) = cr_state & ~UART_CR_UARTEN;

	// set the baud rate
	UART_REG(line, UART_IBRD) = baud_ival;
	UART_REG(line, UART_FBRD) = baud_fval;

	// set the line control registers: 8 bit, no parity, 1 or 2 stop bits, FIFOs enabled/disabled based on line
	UART_REG(line, UART_LCRH) = UART_LCRH_WLEN_HIGH | UART_LCRH_WLEN_LOW | fifo_enable | stop2;

	// Disable all UART interrupts by default
	UART_REG(line, UART_IMSC) = 0;

	// Clear any pending interrupts
	UART_REG(line, UART_ICR) = 0x7FF;

	// re-enable the UART; enable both transmit and receive regardless of previous state
	u32 cr_enable = cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;

	// Enable CTS flow control for Marklin line
	if (line == MARKLIN) {
		cr_enable |= UART_CR_CTSEN;
	}

	UART_REG(line, UART_CR) = cr_enable;
}

inline u8 uart_rx_has_data(size_t line)
{
	return !(UART_REG(line, UART_FR) & UART_FR_RXFE);
}

void uart_puts(size_t line, const char *buf)
{
	if (line == CONSOLE) {
		while (*buf) {
			uart_putc(line, *buf);
			buf++;
		}
	}
}

void uart_putl(size_t line, const char *buf, size_t size)
{
	if (line == CONSOLE) {
		for (size_t i = 0; i < size; i++) {
			uart_putc(line, buf[i]);
		}
	}
}

void uart_printf(size_t line, size_t buf_size, const char *fmt, ...)
{
	if (line != CONSOLE) {
		return;
	}

	va_list va;
	char buf[buf_size];

	va_start(va, fmt);
	__raw_vsnprintf(buf, buf_size, fmt, &va);
	va_end(va);
	uart_puts(line, buf);
}

// Interrupt control functions

void uart_enable_rx_interrupt(size_t line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current | UART_INT_RX | UART_INT_RT;
}

void uart_disable_rx_interrupt(size_t line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current & ~(UART_INT_RX | UART_INT_RT);
}

void uart_enable_tx_interrupt(size_t line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current | UART_INT_TX;
}

void uart_disable_tx_interrupt(size_t line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current & ~UART_INT_TX;
}

void uart_clear_rx_interrupt(size_t line)
{
	UART_REG(line, UART_ICR) = UART_INT_RX | UART_INT_RT;
}

void uart_clear_tx_interrupt(size_t line)
{
	UART_REG(line, UART_ICR) = UART_INT_TX;
}

void uart_clear_ms_interrupt(size_t line)
{
	UART_REG(line, UART_ICR) = UART_INT_MS;
}

void uart_disable_ms_interrupt(size_t line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current & ~UART_INT_MS;
}

u32 uart_get_interrupt_status(size_t line)
{
	return UART_REG(line, UART_MIS);
}

u8 uart_tx_has_space(size_t line)
{
	return !(UART_REG(line, UART_FR) & UART_FR_TXFF);
}

static void uart_kernel_interrupt_handler(u32 irq, void *data)
{
	UNUSED(irq);
	UNUSED(data);

	size_t lines_to_check[] = { CONSOLE, MARKLIN };
	size_t num_lines = sizeof(lines_to_check) / sizeof(lines_to_check[0]);

	for (size_t i = 0; i < num_lines; i++) {
		size_t line = lines_to_check[i];
		u32 status = uart_get_interrupt_status(line);

		if (status == 0) {
			continue;
		}

		// Handle RX interrupts (data available)
		if (status & (UART_INT_RX | UART_INT_RT)) {
			uart_clear_rx_interrupt(line);
			uart_disable_rx_interrupt(line);
			event_unblock_waiting_tasks(EVENT_UART_RX, line);
		}

		// Handle TX interrupts (space available)
		if (status & UART_INT_TX) {
			uart_clear_tx_interrupt(line);
			uart_disable_tx_interrupt(line);
			event_unblock_waiting_tasks(EVENT_UART_TX, line);
		}

		// Handle modem status interrupts (CTS changes) for Marklin
		if ((status & UART_INT_MS) && line == MARKLIN) {
			uart_clear_ms_interrupt(line);
			// We don't want to disable the interrupt here, we want to keep it enabled to catch the next transition
			event_unblock_waiting_tasks(EVENT_UART_MS, line);
		}
	}
}

void uart_init_interrupts(void)
{
	if (interrupt_register_handler(IRQ_UART, uart_kernel_interrupt_handler, NULL) != 0) {
		klog_error("Failed to register UART0 interrupt handler");
	}

	interrupt_set_type(IRQ_UART, IRQ_TYPE_LEVEL_HIGH);
	interrupt_enable(IRQ_UART);
	klog_info("Enabled UART interrupt (IRQ %d)", IRQ_UART);

	klog_info("UART interrupts initialized");
}
