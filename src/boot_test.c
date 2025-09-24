#include "boot_test.h"

#include "compiler.h"
#include "dlist.h"
#include "klog.h"
#include "panic.h"
#include "string.h"
#include "timer/timer.h"
#include "uart.h"
#include "timer/time.h"
#include "types.h"
#include "printf.h"
#include "priority_queue.h"

struct test_data {
	int data;
	struct dlist_node node;
};

static int timer_callback_count = 0;

static void __maybe_unused test_timer_callback(void __maybe_unused *arg)
{
	timer_callback_count++;
}

static void __maybe_unused dlist_test(void)
{
	DLIST_HEAD(list);

	// Test 1: Empty list check
	BUG_ON(!dlist_is_empty(&list));

	struct test_data data1;
	struct test_data data2;
	struct test_data data3;
	struct test_data data4;

	data1.data = 1;
	data2.data = 2;
	data3.data = 3;
	data4.data = 4;

	dlist_init(&data1.node);
	dlist_init(&data2.node);
	dlist_init(&data3.node);
	dlist_init(&data4.node);

	// Test 2: Insert head
	dlist_insert_head(&list, &data4.node);
	BUG_ON(dlist_is_empty(&list));
	BUG_ON(data4.node.next != &list);
	BUG_ON(data4.node.prev != &list);

	// Test 3: Insert multiple nodes
	dlist_insert_head(&list, &data3.node);
	dlist_insert_tail(&list, &data2.node);
	dlist_insert_tail(&list, &data1.node);

	// Test 4: Verify list order (head to tail)
	struct dlist_node *pos;
	DLIST_PRINT(&list, struct test_data, node, "%d", data, 10);
	int expected_values[] = { 3, 4, 2, 1 };
	int i = 0;
	dlist_for_each(pos, &list)
	{
		struct test_data *data = dlist_entry(pos, struct test_data, node);
		BUG_ON(data->data != expected_values[i++]);
	}
	BUG_ON(i != 4); // Verify we processed all nodes

	// Test 5: Verify list order (tail to head)
	int expected_values_reverse[] = { 1, 2, 4, 3 };
	i = 0;
	dlist_for_each_reverse(pos, &list)
	{
		struct test_data *data = dlist_entry(pos, struct test_data, node);
		BUG_ON(data->data != expected_values_reverse[i++]);
	}
	BUG_ON(i != 4); // Verify we processed all nodes

	// Test 6: Test entry iteration macros
	i = 0;
	struct test_data *pos3;
	dlist_for_each_entry(pos3, &list, struct test_data, node)
	{
		BUG_ON(pos3->data != expected_values[i++]);
	}
	BUG_ON(i != 4);

	// Test 7: Test reverse entry iteration macros
	i = 0;
	struct test_data *pos4;
	dlist_for_each_entry_reverse(pos4, &list, struct test_data, node)
	{
		BUG_ON(pos4->data != expected_values_reverse[i++]);
	}
	BUG_ON(i != 4);

	klog_info("All dlist tests passed!");
}

static void __maybe_unused string_test(void)
{
	// Test strcmp
	BUG_ON(strcmp("hello", "hello") != 0);
	BUG_ON(strcmp("hello", "world") >= 0);
	BUG_ON(strcmp("world", "hello") <= 0);

	// Test strncmp
	BUG_ON(strncmp("hello", "help", 3) != 0);
	BUG_ON(strncmp("hello", "help", 4) >= 0);

	// Test strlen
	BUG_ON(strlen("hello") != 5);
	BUG_ON(strlen("") != 0);

	// Test strcpy and strncpy
	char dest[10];
	strcpy(dest, "hello");
	BUG_ON(strcmp(dest, "hello") != 0);

	memset(dest, 0, sizeof(dest));
	strncpy(dest, "hello", 3);
	BUG_ON(dest[3] != '\0'); // Should be null-terminated
	BUG_ON(strncmp(dest, "hel", 3) != 0);

	// Test memcpy
	char src[] = "hello";
	char dst[6];
	memcpy(dst, src, 6);
	BUG_ON(strcmp(dst, src) != 0);

	// Test memmove
	char src2[] = "hello";
	char dst2[6];
	memmove(dst2, src2, 6);
	BUG_ON(strcmp(dst2, src2) != 0);

	// Test strcat
	char dest2[10];
	strcpy(dest2, "Hello");
	strcat(dest2, " World");
	BUG_ON(strcmp(dest2, "Hello World") != 0);

	// Test strncat
	char dest3[10];
	strcpy(dest3, "Hello");
	strncat(dest3, " World", 5);
	BUG_ON(strcmp(dest3, "Hello Worl") != 0);

	klog_info("All string tests passed!");
}

#define WAIT_WITH_TIMERS(ms)                      \
	start = TIME_GET_TICK_MS();               \
	while (TIME_GET_TICK_MS() - start < ms) { \
		timer_process();                  \
	}

static void __maybe_unused timer_test(void)
{
	timer_t test_timer;
	u64 start;
	// Test timer initialization
	timer_init(&test_timer, "test_timer", NULL, NULL);
	BUG_ON(timer_is_active(&test_timer));

	// Test one-shot timer
	timer_start_once(&test_timer, 100);
	BUG_ON(!timer_is_active(&test_timer));

	// Wait for timer to expire
	WAIT_WITH_TIMERS(150);
	BUG_ON(timer_is_active(&test_timer));

	// Test periodic timer
	timer_callback_count = 0;
	timer_init(&test_timer, "test_timer", test_timer_callback, NULL);
	timer_start_periodic(&test_timer, 100);
	BUG_ON(!timer_is_active(&test_timer));

	// Wait for a few periods
	WAIT_WITH_TIMERS(350);
	BUG_ON(timer_callback_count < 3); // Should have triggered at least 3 times

	// Test timer stop
	timer_stop(&test_timer);
	BUG_ON(timer_is_active(&test_timer));

	timer_callback_count = 0;
	// Test with multiple timers
	timer_t test_timer2;
	timer_init(&test_timer2, "test_timer2", test_timer_callback, NULL);
	timer_start_periodic(&test_timer2, 100);
	BUG_ON(!timer_is_active(&test_timer2));

	timer_t test_timer3;
	timer_init(&test_timer3, "test_timer3", test_timer_callback, NULL);
	timer_start_periodic(&test_timer3, 200);
	BUG_ON(!timer_is_active(&test_timer3));

	WAIT_WITH_TIMERS(350);
	BUG_ON(timer_callback_count < 4); // Should have triggered at least 4 times

	timer_stop(&test_timer2);
	BUG_ON(timer_is_active(&test_timer2));

	timer_stop(&test_timer3);
	BUG_ON(timer_is_active(&test_timer3));

	klog_info("All timer tests passed!");
}

static void __maybe_unused printf_test(void)
{
	char buf[1024];
	int ret;

	// Test basic integer formatting
	ret = snprintf(buf, sizeof(buf), "%d", 42);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "42") != 0);

	// Test width and padding
	ret = snprintf(buf, sizeof(buf), "%5d", 42);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "   42") != 0);

	ret = snprintf(buf, sizeof(buf), "%-5d", 42);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "42   ") != 0);

	ret = snprintf(buf, sizeof(buf), "%05d", 42);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "00042") != 0);

	// Test hexadecimal formatting
	ret = snprintf(buf, sizeof(buf), "%x", 42);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "2a") != 0);

	ret = snprintf(buf, sizeof(buf), "%#x", 42);
	BUG_ON(ret != 4);
	BUG_ON(strcmp(buf, "0x2a") != 0);

	ret = snprintf(buf, sizeof(buf), "%X", 42);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "2a") != 0);

	// Test octal formatting
	ret = snprintf(buf, sizeof(buf), "%o", 42);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "52") != 0);

	// Test long integer formatting
	ret = snprintf(buf, sizeof(buf), "%ld", 42L);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "42") != 0);

	ret = snprintf(buf, sizeof(buf), "%lld", 42LL);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "42") != 0);

	// Test string and character
	ret = snprintf(buf, sizeof(buf), "%s", "hello");
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "hello") != 0);

	ret = snprintf(buf, sizeof(buf), "%c", 'A');
	BUG_ON(ret != 1);
	BUG_ON(strcmp(buf, "A") != 0);

	// Test pointer
	void *var = (void *)0x12345678;
	void *var2 = (void *)0x1234;
	ret = snprintf(buf, sizeof(buf), "%p", var);
	BUG_ON(ret != 10);
	BUG_ON(strcmp(buf, "0x12345678") != 0);

	ret = snprintf(buf, sizeof(buf), "%p", var2);
	BUG_ON(ret != 10);
	BUG_ON(strcmp(buf, "0x00001234") != 0);

	// Test string precision
	ret = snprintf(buf, sizeof(buf), "%.3s", "hello");
	BUG_ON(ret != 3);
	BUG_ON(strcmp(buf, "hel") != 0);

	ret = snprintf(buf, sizeof(buf), "%.5s", "hi");
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "hi") != 0);

	ret = snprintf(buf, sizeof(buf), "%.0s", "test");
	BUG_ON(ret != 0);
	BUG_ON(strcmp(buf, "") != 0);

	// Test integer precision
	ret = snprintf(buf, sizeof(buf), "%.5d", 42);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "00042") != 0);

	ret = snprintf(buf, sizeof(buf), "%.3d", 12345);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "12345") != 0);

	ret = snprintf(buf, sizeof(buf), "%.0d", 0);
	BUG_ON(ret != 1);
	BUG_ON(strcmp(buf, "0") != 0);

	// Test hexadecimal precision
	ret = snprintf(buf, sizeof(buf), "%.4x", 0x1A);
	BUG_ON(ret != 4);
	BUG_ON(strcmp(buf, "001a") != 0);

	ret = snprintf(buf, sizeof(buf), "%.2x", 0x1A);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "1a") != 0);

	ret = snprintf(buf, sizeof(buf), "%#.4x", 0x1A);
	BUG_ON(ret != 6);
	BUG_ON(strcmp(buf, "0x001a") != 0);

	// Test octal precision
	ret = snprintf(buf, sizeof(buf), "%.4o", 42);
	BUG_ON(ret != 4);
	BUG_ON(strcmp(buf, "0052") != 0);

	ret = snprintf(buf, sizeof(buf), "%.2o", 42);
	BUG_ON(ret != 2);
	BUG_ON(strcmp(buf, "52") != 0);

	// Test long integer precision
	ret = snprintf(buf, sizeof(buf), "%.5ld", 42L);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "00042") != 0);

	ret = snprintf(buf, sizeof(buf), "%.3ld", 12345L);
	BUG_ON(ret != 5);
	BUG_ON(strcmp(buf, "12345") != 0);

	// Test buffer overflow protection
	ret = snprintf(buf, 5, "hello world");
	BUG_ON(ret != 11); // Should return the length that would have been written
	BUG_ON(strcmp(buf, "hell") != 0); // Should only write up to buffer size - 1

	klog_info("All printf tests passed!");
}

static int work_execution_count = 0;

static void __maybe_unused test_work_fn(void *arg)
{
	int *count = (int *)arg;
	(*count)++;
	work_execution_count++;
}

static void __maybe_unused priority_queue_test(void)
{
	// Test 1: Basic integer priority queue
	PRIORITY_QUEUE_DECLARE(int_pq, int, 10);
	struct int_pq pq;
	pq_init(&pq, pq_compare_int);

	// Test initial state
	BUG_ON(!pq_is_empty(&pq));
	BUG_ON(pq_size(&pq) != 0);
	BUG_ON(pq_capacity(&pq) != 10);
	BUG_ON(pq_is_full(&pq));
	BUG_ON(pq_peek(&pq) != NULL);
	BUG_ON(pq_pop(&pq) != NULL);

	// Test heap validation on empty queue
	BUG_ON(!pq_validate_heap(&pq));

	// Test single element
	int val1 = 5;
	BUG_ON(!pq_push(&pq, &val1));
	BUG_ON(pq_is_empty(&pq));
	BUG_ON(pq_size(&pq) != 1);
	BUG_ON(pq_is_full(&pq));
	BUG_ON(pq_peek(&pq) != &val1);
	BUG_ON(!pq_validate_heap(&pq));

	// Test multiple elements
	int val2 = 3, val3 = 8, val4 = 1, val5 = 7;
	BUG_ON(!pq_push(&pq, &val2));
	BUG_ON(!pq_push(&pq, &val3));
	BUG_ON(!pq_push(&pq, &val4));
	BUG_ON(!pq_push(&pq, &val5));

	BUG_ON(pq_size(&pq) != 5);
	BUG_ON(!pq_validate_heap(&pq));

	// Min element should be at the top
	BUG_ON(*(int *)pq_peek(&pq) != 1);

	// Test popping in sorted order
	int *popped;
	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 1);
	BUG_ON(pq_size(&pq) != 4);
	BUG_ON(!pq_validate_heap(&pq));

	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 3);
	BUG_ON(pq_size(&pq) != 3);
	BUG_ON(!pq_validate_heap(&pq));

	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 5);
	BUG_ON(pq_size(&pq) != 2);
	BUG_ON(!pq_validate_heap(&pq));

	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 7);
	BUG_ON(pq_size(&pq) != 1);
	BUG_ON(!pq_validate_heap(&pq));

	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 8);
	BUG_ON(pq_size(&pq) != 0);
	BUG_ON(!pq_is_empty(&pq));
	BUG_ON(!pq_validate_heap(&pq));

	// Test 2: Test with duplicate values
	int dup1 = 5, dup2 = 5, dup3 = 3;
	BUG_ON(!pq_push(&pq, &dup1));
	BUG_ON(!pq_push(&pq, &dup2));
	BUG_ON(!pq_push(&pq, &dup3));
	BUG_ON(!pq_validate_heap(&pq));

	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 3);
	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 5);
	popped = (int *)pq_pop(&pq);
	BUG_ON(*popped != 5);
	BUG_ON(!pq_is_empty(&pq));

	// Test 3: Test with full queue
	for (int i = 0; i < 10; i++) {
		static int values[10] = { 9, 2, 6, 1, 8, 3, 7, 4, 0, 5 };
		BUG_ON(!pq_push(&pq, &values[i]));
	}
	BUG_ON(!pq_is_full(&pq));
	BUG_ON(pq_size(&pq) != 10);

	int overflow_val = 99;
	BUG_ON(pq_push(&pq, &overflow_val));
	BUG_ON(pq_size(&pq) != 10);

	int last_val = -1;
	for (int i = 0; i < 10; i++) {
		popped = (int *)pq_pop(&pq);
		BUG_ON(*popped < last_val);
		last_val = *popped;
	}
	BUG_ON(!pq_is_empty(&pq));

	// Test 4
	int clear_val1 = 1, clear_val2 = 2, clear_val3 = 3;
	BUG_ON(!pq_push(&pq, &clear_val1));
	BUG_ON(!pq_push(&pq, &clear_val2));
	BUG_ON(!pq_push(&pq, &clear_val3));
	BUG_ON(pq_size(&pq) != 3);

	pq_clear(&pq);
	BUG_ON(!pq_is_empty(&pq));
	BUG_ON(pq_size(&pq) != 0);
	BUG_ON(pq_peek(&pq) != NULL);

	// Test 5: Test with pointer
	PRIORITY_QUEUE_DECLARE(ptr_pq, void, 5);
	struct ptr_pq ptr_queue;
	pq_init(&ptr_queue, pq_compare_ptr);

	void *ptr1 = (void *)0x1000;
	void *ptr2 = (void *)0x2000;
	void *ptr3 = (void *)0x1500;
	void *ptr4 = (void *)0x0500;

	BUG_ON(!pq_push(&ptr_queue, ptr1));
	BUG_ON(!pq_push(&ptr_queue, ptr2));
	BUG_ON(!pq_push(&ptr_queue, ptr3));
	BUG_ON(!pq_push(&ptr_queue, ptr4));

	BUG_ON(!pq_validate_heap(&ptr_queue));

	// Should pop pointers in address order (lowest first)
	void *popped_ptr;
	popped_ptr = pq_pop(&ptr_queue);
	BUG_ON(popped_ptr != ptr4); // 0x0500

	popped_ptr = pq_pop(&ptr_queue);
	BUG_ON(popped_ptr != ptr1); // 0x1000

	popped_ptr = pq_pop(&ptr_queue);
	BUG_ON(popped_ptr != ptr3); // 0x1500

	popped_ptr = pq_pop(&ptr_queue);
	BUG_ON(popped_ptr != ptr2); // 0x2000

	BUG_ON(!pq_is_empty(&ptr_queue));

	klog_info("All priority queue tests passed!");
}

#ifdef DEBUG_BUILD
void boot_test(void)
{
	klog_info("Boot test started");
	uart_process_tx_buffers_blocking();
	dlist_test();
	string_test();
	timer_test();
	printf_test();
	priority_queue_test();
	klog_info("Boot test passed!");
}
#else
void boot_test(void)
{
}
#endif
