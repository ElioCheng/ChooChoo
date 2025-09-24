#include "srr_perf.h"
#include "syscall.h"
#include "uart.h"

#define NUM_ITERATIONS 10000
#define WARMUP_ITERATIONS 100
#define MAX_MSG_SIZE 256

// Test configurations
#ifdef OPTIMIZATION
#define OPT_STR "opt"
#else
#define OPT_STR "noopt"
#endif

#ifdef ICACHE_ONLY
#define CACHE_STR "icache"
#elif defined(DCACHE_ONLY)
#define CACHE_STR "dcache"
#elif defined(BOTH_CACHE)
#define CACHE_STR "bcache"
#else
#define CACHE_STR "nocache"
#endif

static int sender_tid = -1;
static int receiver_tid = -1;
static int current_msg_size = 4;
static int receiver_first = 0;

static void print_csv_row(const char *opt, const char *cache, const char *order, int msg_size, u64 time_us, int iterations)
{
	console_printf("%s,%s,%s,%d,%d,%d\r\n", opt, cache, order, msg_size, time_us, iterations);
}

static void get_message_with_size(int size, char *buffer)
{
	for (int i = 0; i < size; i++) {
		buffer[i] = 'A' + (i % 26); // Fill with some data
	}
}

void sender_task()
{
	char sender_buffer[MAX_MSG_SIZE] = { 0 };
	char reply_buffer[MAX_MSG_SIZE] = { 0 };
	get_message_with_size(current_msg_size, sender_buffer);

	// Warmup phase to heat up the cache
	for (int i = 0; i < WARMUP_ITERATIONS; i++) {
		Send(receiver_tid, sender_buffer, current_msg_size, reply_buffer, MAX_MSG_SIZE);
	}

	u64 start_time = time_get_tick_64();

	for (int i = 0; i < NUM_ITERATIONS; i++) {
		Send(receiver_tid, sender_buffer, current_msg_size, reply_buffer, MAX_MSG_SIZE);
	}

	u64 end_time = time_get_tick_64();
	u64 total_time_us = (end_time - start_time);

	const char *order_str = receiver_first ? "R" : "S";
	print_csv_row(OPT_STR, CACHE_STR, order_str, current_msg_size, total_time_us, NUM_ITERATIONS);

	Exit();
}

void receiver_task()
{
	char receiver_buffer[MAX_MSG_SIZE] = { 0 };
	char reply_buffer[MAX_MSG_SIZE] = { 0 };
	int sender_tid_recv;
	get_message_with_size(current_msg_size, receiver_buffer);
	get_message_with_size(current_msg_size, reply_buffer);

	for (int i = 0; i < WARMUP_ITERATIONS; i++) {
		Receive(&sender_tid_recv, receiver_buffer, current_msg_size);
		Reply(sender_tid_recv, reply_buffer, current_msg_size);
	}

	for (int i = 0; i < NUM_ITERATIONS; i++) {
		Receive(&sender_tid_recv, receiver_buffer, current_msg_size);
		Reply(sender_tid_recv, reply_buffer, current_msg_size);
	}

	Exit();
}

void run_test(int msg_size, int recv_first)
{
	current_msg_size = msg_size;
	receiver_first = recv_first;

	if (receiver_first) {
		receiver_tid = Create(6, receiver_task);
		sender_tid = Create(7, sender_task);
	} else {
		sender_tid = Create(6, sender_task);
		receiver_tid = Create(7, receiver_task);
	}

	WaitTid(sender_tid);
	WaitTid(receiver_tid);
}

void srr_perf_main()
{
	int msg_sizes[] = { 4, 64, 256 };
	int i, j;

	console_printf("optimization,cache,order,msgsize,total_time_us,iterations\r\n");

	for (i = 0; i < 3; i++) { // 3 message sizes
		for (j = 0; j < 2; j++) { // 2 execution orders
			run_test(msg_sizes[i], j); // j=0: sender_first, j=1: receiver_first
		}
	}

	Exit();
}
