#include "io_server.h"
#include "clock.h"
#include "syscall.h"
#include "name.h"
#include "io.h"
#include "event.h"
#include "klog.h"
#include "compiler.h"
#include "string.h"

// ############################################################################
// UART hardware access
// ############################################################################
#ifndef MMIO_BASE
#define MMIO_BASE (char *)0xFE000000
#endif

#define UART0_BASE (char *)(MMIO_BASE + 0x201000)
#define UART3_BASE (char *)(MMIO_BASE + 0x201600)

#define UART_REG(line, offset) (*(volatile u32 *)(io_state.line_uarts[line] + offset))

// UART register offsets
#define UART_DR 0x00
#define UART_FR 0x18
#define UART_IMSC 0x38 // Interrupt Mask Set Clear register
#define UART_ICR 0x44 // Interrupt Clear register

// UART flags
#define UART_FR_RXFE 0x10
#define UART_FR_TXFF 0x20
#define UART_FR_TXFE 0x80

// UART interrupt bits
#define UART_INT_RX 0x10 // Receive interrupt
#define UART_INT_TX 0x20 // Transmit interrupt
#define UART_INT_RT 0x40 // Receive timeout interrupt
#define UART_INT_MS 0x01 // Modem status interrupt (includes CTS changes)

// Global IO server state
static io_server_state_t io_state;

// ############################################################################
// State initialization
// ############################################################################
static void init_io_server_state(void)
{
	// Initialize UART bases
	io_state.line_uarts[0] = NULL;
	io_state.line_uarts[IO_CHANNEL_CONSOLE] = UART0_BASE;
	io_state.line_uarts[IO_CHANNEL_MARKLIN] = UART3_BASE;

	// Initialize TX interrupt state
	for (int i = 0; i < 3; i++) {
		io_state.waiting_for_tx_interrupt[i] = false;
	}

	// Initialize console buffer
	memset(io_state.console_tx_buffer, 0, CONSOLE_TX_BUFFER_SIZE);
	io_state.console_tx_head = 0;
	io_state.console_tx_tail = 0;
	io_state.console_tx_count = 0;

	// Initialize client pool
	dlist_init(&io_state.free_clients);
	for (int i = 0; i < IO_SERVER_MAX_CLIENTS; i++) {
		dlist_insert_tail(&io_state.free_clients, &io_state.client_pool[i].node);
	}

	// Initialize operation queues
	dlist_init(&io_state.console_rx_queue);
	dlist_init(&io_state.marklin_rx_queue);
	dlist_init(&io_state.console_tx_queue);
	dlist_init(&io_state.marklin_tx_queue);
}

// Forward declaration for functions used in race condition handling
static void handle_tx_notify(int channel);

static void io_uart_enable_rx_interrupt(int line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current | UART_INT_RX | UART_INT_RT;
}

static void io_uart_enable_tx_interrupt(int line)
{
	u32 current = UART_REG(line, UART_IMSC);
	UART_REG(line, UART_IMSC) = current | UART_INT_TX;
	io_state.waiting_for_tx_interrupt[line] = true;
}

static u8 io_uart_rx_has_data(int line)
{
	return !(UART_REG(line, UART_FR) & UART_FR_RXFE);
}

static u8 io_uart_tx_has_space(int line)
{
	return !(UART_REG(line, UART_FR) & UART_FR_TXFF);
}

static bool io_marklin_can_transmit(void)
{
	// Check TX ready
	if (!io_uart_tx_has_space(IO_CHANNEL_MARKLIN)) {
		return false;
	}
	return true;
}

static unsigned char io_uart_getc(int line)
{
	if (!io_uart_rx_has_data(line)) {
		return 0;
	}
	return UART_REG(line, UART_DR);
}

static int io_marklin_putc_direct(char c)
{
	if (!io_marklin_can_transmit()) {
		return IO_ERROR;
	}

	UART_REG(IO_CHANNEL_MARKLIN, UART_DR) = c;

	return IO_SUCCESS;
}

// ############################################################################
// Console buffer
// ############################################################################

static __maybe_unused int console_buffer_add(char c)
{
	if (io_state.console_tx_count >= CONSOLE_TX_BUFFER_SIZE) {
		return IO_ERROR; // Buffer full
	}

	io_state.console_tx_buffer[io_state.console_tx_head] = c;
	io_state.console_tx_head = (io_state.console_tx_head + 1) % CONSOLE_TX_BUFFER_SIZE;
	io_state.console_tx_count++;
	return IO_SUCCESS;
}

static int console_buffer_flush(int line)
{
	int flushed = 0;

	// Flush as many characters as possible without waiting
	while (io_state.console_tx_count > 0 && io_uart_tx_has_space(line)) {
		UART_REG(line, UART_DR) = io_state.console_tx_buffer[io_state.console_tx_tail];
		io_state.console_tx_tail = (io_state.console_tx_tail + 1) % CONSOLE_TX_BUFFER_SIZE;
		io_state.console_tx_count--;
		flushed++;
	}

	return flushed;
}

static int io_uart_putc(int line, char c)
{
	// Console uses buffering, Marklin uses direct transmission
	if (line == IO_CHANNEL_CONSOLE) {
		console_buffer_flush(line);

		// If buffer is empty, try direct transmission
		if (io_state.console_tx_count == 0 && io_uart_tx_has_space(line)) {
			UART_REG(line, UART_DR) = c;
			return IO_SUCCESS;
		}

		if (console_buffer_add(c) == IO_SUCCESS) {
			console_buffer_flush(line);

			// If there's still data in buffer and we're not already waiting for TX interrupt
			if (io_state.console_tx_count > 0 && !io_state.waiting_for_tx_interrupt[line]) {
				io_uart_enable_tx_interrupt(line);
			}
			return IO_SUCCESS;
		} else {
			return IO_ERROR; // Buffer full
		}
	} else {
		return io_marklin_putc_direct(c);
	}
}

// ############################################################################
// Client pool
// ############################################################################
static io_client_t *alloc_client(void)
{
	if (dlist_is_empty(&io_state.free_clients)) {
		return NULL;
	}

	struct dlist_node *node = dlist_first(&io_state.free_clients);
	dlist_del(node);
	io_client_t *client = dlist_entry(node, io_client_t, node);

	client->tid = -1;
	client->channel = -1;
	client->pending_char = 0;
	dlist_init_node(&client->node);

	return client;
}

static void free_client(io_client_t *client)
{
	dlist_insert_tail(&io_state.free_clients, &client->node);
}

// ############################################################################
// Track pending operations for each channel
// ############################################################################
static struct dlist_node *get_rx_queue(int channel)
{
	switch (channel) {
	case IO_CHANNEL_CONSOLE:
		return &io_state.console_rx_queue;
	case IO_CHANNEL_MARKLIN:
		return &io_state.marklin_rx_queue;
	default:
		return NULL;
	}
}

static struct dlist_node *get_tx_queue(int channel)
{
	switch (channel) {
	case IO_CHANNEL_CONSOLE:
		return &io_state.console_tx_queue;
	case IO_CHANNEL_MARKLIN:
		return &io_state.marklin_tx_queue;
	default:
		return NULL;
	}
}

// ############################################################################
// Handle requests
// ############################################################################
static int handle_getc(int sender_tid, int channel)
{
	if (channel != IO_CHANNEL_CONSOLE && channel != IO_CHANNEL_MARKLIN) {
		return IO_ERROR;
	}

	// Try to get character immediately
	if (io_uart_rx_has_data(channel)) {
		unsigned char c = io_uart_getc(channel);
		return (int)c;
	}

	// No data available, queue the client and enable RX interrupt
	io_client_t *client = alloc_client();
	if (!client) {
		return IO_ERROR;
	}

	client->tid = sender_tid;
	client->channel = channel;

	struct dlist_node *queue = get_rx_queue(channel);
	if (unlikely(!queue)) {
		free_client(client);
		return IO_ERROR;
	}

	dlist_insert_tail(queue, &client->node);

	io_uart_enable_rx_interrupt(channel);

	return IO_BLOCKED;
}

static int handle_trygetc(int sender_tid, int channel)
{
	(void)sender_tid;

	if (channel != IO_CHANNEL_CONSOLE && channel != IO_CHANNEL_MARKLIN) {
		return IO_ERROR;
	}

	// Try to get character immediately
	if (io_uart_rx_has_data(channel)) {
		unsigned char c = io_uart_getc(channel);
		return (int)c;
	}

	return IO_NO_DATA;
}

static int handle_putc(int sender_tid, int channel, unsigned char ch)
{
	if (unlikely(channel != IO_CHANNEL_CONSOLE && channel != IO_CHANNEL_MARKLIN)) {
		return IO_ERROR;
	}

	// Try to transmit immediately (console uses buffering, Marklin uses direct TX)
	int result = io_uart_putc(channel, ch);
	if (result == IO_SUCCESS) {
		return IO_SUCCESS;
	}

	// Transmission failed, need to wait for TX interrupt (only for Marklin)
	if (channel == IO_CHANNEL_MARKLIN) {
		io_client_t *client = alloc_client();
		if (!client) {
			return IO_ERROR;
		}

		client->tid = sender_tid;
		client->channel = channel;
		client->pending_char = ch;

		struct dlist_node *queue = get_tx_queue(channel);
		if (unlikely(!queue)) {
			free_client(client);
			return IO_ERROR;
		}

		dlist_insert_tail(queue, &client->node);

		io_uart_enable_tx_interrupt(channel);

		return IO_BLOCKED;
	}

	return IO_ERROR;
}

static int handle_putn(int sender_tid, int channel, const char *str, size_t len)
{
	(void)sender_tid;

	if (channel != IO_CHANNEL_CONSOLE && channel != IO_CHANNEL_MARKLIN) {
		return IO_ERROR;
	}

	if (len == 0) {
		return 0;
	}

	if (len > IO_REQ_PUTN_MAX_LEN) {
		return IO_ERROR;
	}

	size_t transmitted = 0;

	for (size_t i = 0; i < len; i++) {
		int result = io_uart_putc(channel, str[i]);
		if (result == IO_SUCCESS) {
			transmitted++;
		} else {
			break;
		}
	}

	return (int)transmitted;
}

static void handle_rx_notify(int channel)
{
	struct dlist_node *queue = get_rx_queue(channel);
	if (unlikely(!queue) || dlist_is_empty(queue)) {
		return;
	}

	struct dlist_node *pos, *n;
	dlist_for_each_safe(pos, n, queue)
	{
		if (!io_uart_rx_has_data(channel)) {
			break;
		}

		io_client_t *client = dlist_entry(pos, io_client_t, node);
		unsigned char c = io_uart_getc(channel);

		io_reply_t reply;
		reply.result = (int)c;
		Reply(client->tid, (const char *)&reply, sizeof(reply));

		dlist_del(&client->node);
		free_client(client);
	}

	if (!dlist_is_empty(queue)) {
		io_uart_enable_rx_interrupt(channel);
	}
}

static void handle_tx_notify(int channel)
{
	// Interrupt is disabled in the kernel handler
	io_state.waiting_for_tx_interrupt[channel] = false;

	if (channel == IO_CHANNEL_CONSOLE) {
		console_buffer_flush(channel);

		if (io_state.console_tx_count > 0) {
			io_uart_enable_tx_interrupt(channel);
		}
	} else if (channel == IO_CHANNEL_MARKLIN) {
		struct dlist_node *queue = get_tx_queue(channel);
		if (queue && !dlist_is_empty(queue)) {
			struct dlist_node *pos, *n;
			dlist_for_each_safe(pos, n, queue)
			{
				io_client_t *client = dlist_entry(pos, io_client_t, node);

				if (io_marklin_can_transmit()) {
					io_reply_t reply;
					reply.result = IO_SUCCESS;
					Reply(client->tid, (const char *)&reply, sizeof(reply));

					dlist_del(&client->node);
					free_client(client);

					UART_REG(channel, UART_DR) = client->pending_char;

					break;
				} else {
					io_uart_enable_tx_interrupt(channel);
					break;
				}
			}
		}
	}
}

// ############################################################################
// IO server task entry points
// ############################################################################
void io_server_task(void)
{
	int sender_tid;
	io_request_t request;
	io_reply_t reply;

	init_io_server_state();

	RegisterAs(IO_SERVER_NAME);

	Create(IO_SERVER_PRIORITY - 1, io_rx_notifier_task);
	Create(IO_SERVER_PRIORITY - 1, io_tx_notifier_task);

	klog_info("IO Server started");

	while (1) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));
		if (result < 0) {
			klog_error("IO Server: Receive error");
			continue;
		}
		switch (request.type) {
		case IO_REQ_GETC:
			reply.result = handle_getc(sender_tid, request.channel);
			if (reply.result != IO_BLOCKED) {
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
			break;

		case IO_REQ_TRYGETC:
			reply.result = handle_trygetc(sender_tid, request.channel);
			if (reply.result != IO_BLOCKED) {
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
			break;

		case IO_REQ_PUTC:
			reply.result = handle_putc(sender_tid, request.channel, request.putc.ch);
			if (reply.result != IO_BLOCKED) {
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
			break;

		case IO_REQ_PUTN:
			reply.result = handle_putn(sender_tid, request.channel, request.putn.str, request.putn.len);
			if (reply.result != IO_BLOCKED) {
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
			break;

		case IO_REQ_RX_NOTIFY:
			handle_rx_notify(request.notify.channel);
			reply.result = IO_SUCCESS;
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;

		case IO_REQ_TX_NOTIFY:
			handle_tx_notify(request.notify.channel);
			reply.result = IO_SUCCESS;
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;

		default:
			klog_error("IO Server: Unknown request type");
			reply.result = IO_ERROR;
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;
		}
	}
}

// RX Notifier task - waits for RX interrupts and notifies main server
void io_rx_notifier_task(void)
{
	int io_tid = WhoIs(IO_SERVER_NAME);
	io_request_t request;
	io_reply_t reply;

	klog_info("IO RX Notifier task started");

	while (1) {
		int event_data = AwaitEvent(EVENT_UART_RX);

		// event_data should contain the UART line number
		if (event_data == 1 || event_data == 2) {
			request.type = IO_REQ_RX_NOTIFY;
			request.notify.channel = event_data;

			Send(io_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
		}
	}
}

// TX Notifier task - waits for TX interrupts and notifies main server
void io_tx_notifier_task(void)
{
	int io_tid = WhoIs(IO_SERVER_NAME);
	io_request_t request;
	io_reply_t reply;

	klog_info("IO TX Notifier task started");

	while (1) {
		int event_data = AwaitEvent(EVENT_UART_TX);

		// event_data should contain the UART line number
		if (event_data == 1 || event_data == 2) {
			request.type = IO_REQ_TX_NOTIFY;
			request.notify.channel = event_data;

			Send(io_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
		}
	}
}

// ############################################################################
// Busy-wait debug functions
// ############################################################################
#ifdef ENABLE_BUSY_WAIT_DEBUG

static int busy_wait_uart_putc_direct(int line, char c)
{
	if (line != IO_CHANNEL_CONSOLE && line != IO_CHANNEL_MARKLIN) {
		return IO_ERROR;
	}

	if (line == IO_CHANNEL_CONSOLE) {
		while (!io_uart_tx_has_space(line)) {
			// Busy wait
		}
		UART_REG(line, UART_DR) = c;

		while (!(UART_REG(line, UART_FR) & UART_FR_TXFE)) {
			// Busy wait
		}
		return IO_SUCCESS;
	}

	if (line == IO_CHANNEL_MARKLIN) {
		while (!io_uart_tx_has_space(line)) {
			// Busy wait
		}

		UART_REG(line, UART_DR) = c;

		return IO_SUCCESS;
	}

	return IO_ERROR;
}

int busy_wait_console_putc(char c)
{
	return busy_wait_uart_putc_direct(IO_CHANNEL_CONSOLE, c);
}

int busy_wait_console_puts(const char *str)
{
	if (!str) {
		return IO_ERROR;
	}

	int count = 0;
	while (*str) {
		if (busy_wait_console_putc(*str) != IO_SUCCESS) {
			return count;
		}
		str++;
		count++;
	}
	return count;
}

int busy_wait_marklin_putc(char c)
{
	return busy_wait_uart_putc_direct(IO_CHANNEL_MARKLIN, c);
}

int busy_wait_marklin_putn(const char *str, size_t len)
{
	if (!str || len == 0) {
		return 0;
	}

	int count = 0;
	for (size_t i = 0; i < len; i++) {
		if (busy_wait_marklin_putc(str[i]) != IO_SUCCESS) {
			return count;
		}
		count++;
	}
	return count;
}

#endif // ENABLE_BUSY_WAIT_DEBUG
