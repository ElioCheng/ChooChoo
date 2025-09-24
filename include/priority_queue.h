#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <stddef.h>
#include "types.h"

/**
 * Generic min-heap based priority queue implementation
 *
 * Usage:
 * 1. Define a comparison function for your type, or use the provided pq_compare_int or pq_compare_ptr
 * 2. Declare a priority queue with PRIORITY_QUEUE_DECLARE or DECLARE_PRIORITY_QUEUE
 * 3. Initialize with pq_init
 * 4. Use pq_push, pq_pop, pq_peek, pq_is_empty
 */

#define PRIORITY_QUEUE_DECLARE(name, type, max_size)          \
	struct name {                                         \
		type *items[max_size];                        \
		int size;                                     \
		int (*compare)(const type *a, const type *b); \
	}

#define PRIORITY_QUEUE_INIT(pq, cmp_func)   \
	do {                                \
		(pq)->size = 0;             \
		(pq)->compare = (cmp_func); \
	} while (0)

// Generic priority queue operations
#define pq_init(pq, cmp_func) PRIORITY_QUEUE_INIT(pq, cmp_func)
#define pq_is_empty(pq) ((pq)->size == 0)
#define pq_size(pq) ((pq)->size)
#define pq_peek(pq) ((pq)->size > 0 ? (pq)->items[0] : NULL)

#define PQ_PARENT(i) (((i) - 1) / 2)
#define PQ_LEFT_CHILD(i) (2 * (i) + 1)
#define PQ_RIGHT_CHILD(i) (2 * (i) + 2)

#define pq_push(pq, item)                                                               \
	({                                                                              \
		bool success = false;                                                   \
		if ((pq)->size < (int)(sizeof((pq)->items) / sizeof((pq)->items[0]))) { \
			(pq)->items[(pq)->size] = (item);                               \
			(pq)->size++;                                                   \
			_pq_bubble_up(pq, (pq)->size - 1);                              \
			success = true;                                                 \
		}                                                                       \
		success;                                                                \
	})

#define pq_pop(pq)                                                    \
	({                                                            \
		typeof((pq)->items[0]) result = NULL;                 \
		if ((pq)->size > 0) {                                 \
			result = (pq)->items[0];                      \
			(pq)->items[0] = (pq)->items[(pq)->size - 1]; \
			(pq)->size--;                                 \
			if ((pq)->size > 0) {                         \
				_pq_bubble_down(pq, 0);               \
			}                                             \
		}                                                     \
		result;                                               \
	})

#define _pq_bubble_up(pq, idx)                                                             \
	do {                                                                               \
		int current = (idx);                                                       \
		while (current > 0) {                                                      \
			int parent = PQ_PARENT(current);                                   \
			if ((pq)->compare((pq)->items[parent], (pq)->items[current]) <= 0) \
				break;                                                     \
			_pq_swap_items(pq, parent, current);                               \
			current = parent;                                                  \
		}                                                                          \
	} while (0)

#define _pq_bubble_down(pq, idx)                                                                                  \
	do {                                                                                                      \
		int current = (idx);                                                                              \
		while (true) {                                                                                    \
			int left = PQ_LEFT_CHILD(current);                                                        \
			int right = PQ_RIGHT_CHILD(current);                                                      \
			int smallest = current;                                                                   \
                                                                                                                  \
			if (left < (pq)->size && (pq)->compare((pq)->items[left], (pq)->items[smallest]) < 0) {   \
				smallest = left;                                                                  \
			}                                                                                         \
			if (right < (pq)->size && (pq)->compare((pq)->items[right], (pq)->items[smallest]) < 0) { \
				smallest = right;                                                                 \
			}                                                                                         \
                                                                                                                  \
			if (smallest == current)                                                                  \
				break;                                                                            \
                                                                                                                  \
			_pq_swap_items(pq, current, smallest);                                                    \
			current = smallest;                                                                       \
		}                                                                                                 \
	} while (0)

#define _pq_swap_items(pq, i, j)                              \
	do {                                                  \
		typeof((pq)->items[0]) temp = (pq)->items[i]; \
		(pq)->items[i] = (pq)->items[j];              \
		(pq)->items[j] = temp;                        \
	} while (0)

#define pq_clear(pq)            \
	do {                    \
		(pq)->size = 0; \
	} while (0)

#define pq_capacity(pq) ((int)(sizeof((pq)->items) / sizeof((pq)->items[0])))

#define pq_is_full(pq) ((pq)->size >= pq_capacity(pq))

#define pq_validate_heap(pq)                                                                               \
	({                                                                                                 \
		bool valid = true;                                                                         \
		for (int i = 0; i < (pq)->size && valid; i++) {                                            \
			int left = PQ_LEFT_CHILD(i);                                                       \
			int right = PQ_RIGHT_CHILD(i);                                                     \
			if (left < (pq)->size && (pq)->compare((pq)->items[i], (pq)->items[left]) > 0) {   \
				valid = false;                                                             \
			}                                                                                  \
			if (right < (pq)->size && (pq)->compare((pq)->items[i], (pq)->items[right]) > 0) { \
				valid = false;                                                             \
			}                                                                                  \
		}                                                                                          \
		valid;                                                                                     \
	})

#define DECLARE_PRIORITY_QUEUE(name, type, max_size) PRIORITY_QUEUE_DECLARE(name, type, max_size) name

#define PQ_DEBUG_PRINT_ARRAY(pq, format_func, print_func)                        \
	do {                                                                     \
		print_func("Priority Queue Array [size=%d]: ", (pq)->size);      \
		for (int _i = 0; _i < (pq)->size; _i++) {                        \
			print_func("[%d] %s", _i, format_func((pq)->items[_i])); \
		}                                                                \
		print_func("\n");                                                \
	} while (0)

static inline int pq_compare_int(const int *a, const int *b)
{
	return (*a < *b) ? -1 : ((*a > *b) ? 1 : 0);
}

static inline int pq_compare_ptr(const void *a, const void *b)
{
	return (a < b) ? -1 : ((a > b) ? 1 : 0);
}

#endif /* PRIORITY_QUEUE_H */
