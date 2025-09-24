#ifndef IO_SERVER_H
#define IO_SERVER_H

#include "types.h"
#include "dlist.h"
#include "io.h"
#include "params.h"

#define IO_SERVER_NAME "io_server"
#define IO_SERVER_PRIORITY 2
#define IO_SERVER_MAX_CLIENTS MAX_TASKS

// Console buffer size
#define CONSOLE_TX_BUFFER_SIZE 10240 // 10KB

// CTS Flow Control State Machine
typedef enum {
	CTS_STATE_IDLE, // Ready to send, waiting for TX & CTS both up
	CTS_STATE_SENT, // Just sent a byte, waiting for CTS to go down
	CTS_STATE_CTS_DOWN, // CTS went down, waiting for CTS to go back up
	CTS_STATE_READY // CTS back up, ready for next transmission
} cts_state_t;

// blocked io clients
typedef struct io_client {
	int tid;
	int channel;
	unsigned char pending_char; // For TX operations
	struct dlist_node node;
} io_client_t;

// IO Server state structure
typedef struct io_server_state {
	// UART hardware access
	char *line_uarts[3];
	bool waiting_for_tx_interrupt[3];

	// Console buffer
	char console_tx_buffer[CONSOLE_TX_BUFFER_SIZE];
	size_t console_tx_head;
	size_t console_tx_tail;
	size_t console_tx_count;

	// Client pool
	io_client_t client_pool[IO_SERVER_MAX_CLIENTS];
	struct dlist_node free_clients;

	// Pending operation queues
	struct dlist_node console_rx_queue;
	struct dlist_node marklin_rx_queue;
	struct dlist_node console_tx_queue;
	struct dlist_node marklin_tx_queue;
} io_server_state_t;

void io_server_task(void);
void io_rx_notifier_task(void);
void io_tx_notifier_task(void);
void io_cts_notifier_task(void);

#endif // IO_SERVER_H
