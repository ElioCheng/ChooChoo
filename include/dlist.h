#ifndef DLIST_H
#define DLIST_H

#ifdef __KERNEL__
#include "klog.h"
#include "panic.h"
#endif // __KERNEL__

#include <stddef.h>
#include "types.h"
struct dlist_node {
	struct dlist_node *prev;
	struct dlist_node *next;
};

#define DLIST_HEAD(name) struct dlist_node name = { &(name), &(name) }
#define DLIST_HEAD_INIT(name) { &(name), &(name) }

// Initialize a list head
static inline void dlist_init(struct dlist_node *head)
{
	head->prev = head;
	head->next = head;
}

// Initialize a node
static inline void dlist_init_node(struct dlist_node *node)
{
	node->prev = node;
	node->next = node;
}

// Get the first node of the list
static inline struct dlist_node *dlist_first(struct dlist_node *head)
{
	return head->next;
}

// Get the last node of the list
static inline struct dlist_node *dlist_last(struct dlist_node *head)
{
	return head->prev;
}

// Check if the list is empty
static inline bool dlist_is_empty(struct dlist_node *head)
{
	return head->next == head;
}

// Insert a node after the given position
static inline void dlist_insert(struct dlist_node *pos, struct dlist_node *node)
{
#ifdef __KERNEL__
	if (pos == node) {
		// klog_warning("dlist_insert: pos(%p) == node(%p)", pos, node);
		panic("dlist_insert: pos(%p) == node(%p)", pos, node);
	}
#endif // __KERNEL__
	node->prev = pos;
	node->next = pos->next;
	pos->next->prev = node;
	pos->next = node;
}

// Add a node to the start of the list
static inline void dlist_insert_head(struct dlist_node *head, struct dlist_node *node)
{
	dlist_insert(head, node);
}

// Insert a node to the tail of the list
static inline void dlist_insert_tail(struct dlist_node *head, struct dlist_node *node)
{
	struct dlist_node *tmp;

	tmp = dlist_last(head);

	dlist_insert(tmp, node);
}

// Delete a node from the list
static inline void dlist_del(struct dlist_node *node)
{
	node->prev->next = node->next;
	node->next->prev = node->prev;
	node->next = node;
	node->prev = node;
}

// Move a node to the end of the list
static inline void dlist_move(struct dlist_node *node, struct dlist_node *head)
{
	dlist_del(node);
	dlist_insert_tail(head, node);
}

// Replace a node with another node
static inline void dlist_replace(struct dlist_node *old, struct dlist_node *new)
{
	new->next = old->next;
	new->next->prev = new;
	new->prev = old->prev;
	new->prev->next = new;
}

// Swap two nodes
static inline void dlist_swap(struct dlist_node *entry1, struct dlist_node *entry2)
{
	struct dlist_node *pos = entry2->prev;

	dlist_del(entry2);
	dlist_replace(entry1, entry2);
	if (pos == entry1)
		pos = entry2;
	dlist_insert(pos, entry1);
}

#define dlist_entry(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))

#define dlist_for_each(pos, head) for (pos = (head)->next; pos != (head); pos = pos->next)

#define dlist_for_each_reverse(pos, head) for (pos = (head)->prev; pos != (head); pos = pos->prev)

// Safe iteration means that the list can be modified during the iteration
#define dlist_for_each_safe(pos, n, head) for (pos = (head)->next, n = pos->next; pos != (head); pos = n, n = pos->next)

// Safe reverse iteration means that the list can be modified during the iteration
#define dlist_for_each_reverse_safe(pos, n, head) \
	for (pos = (head)->prev, n = pos->prev; pos != (head); pos = n, n = pos->prev)

#define dlist_for_each_entry(pos, head, type, member)                               \
	for (pos = dlist_entry((head)->next, type, member); &pos->member != (head); \
	     pos = dlist_entry(pos->member.next, type, member))

#define dlist_for_each_entry_reverse(pos, head, type, member)                       \
	for (pos = dlist_entry((head)->prev, type, member); &pos->member != (head); \
	     pos = dlist_entry(pos->member.prev, type, member))

#define dlist_for_each_entry_safe(pos, n, head, type, member)                                             \
	for (pos = dlist_entry((head)->next, type, member), n = pos->member.next; &pos->member != (head); \
	     pos = dlist_entry(n, type, member), n = pos->member.next)

#define dlist_for_each_entry_safe_reverse(pos, n, head, type, member)                                     \
	for (pos = dlist_entry((head)->prev, type, member), n = pos->member.prev; &pos->member != (head); \
	     pos = dlist_entry(n, type, member), n = pos->member.prev)

#define dlist_len(head)                                                    \
	({                                                                 \
		struct dlist_node *pos;                                    \
		int len = 0;                                               \
		for (pos = (head)->next; pos != (head); pos = pos->next) { \
			len++;                                             \
		}                                                          \
		len;                                                       \
	})

#ifdef __KERNEL__
#define DLIST_PRINT(head, type, member, field_fmt, field, max_len)                                        \
	do {                                                                                              \
		u64 __dlist_print_len = 0;                                                                \
		struct dlist_node *__dlist_print_pos;                                                     \
		if (dlist_is_empty(head)) {                                                               \
			klog_debug("dlist_print: " field_fmt, "empty");                                   \
		} else {                                                                                  \
			for (__dlist_print_pos = (head)->next; __dlist_print_pos != (head);               \
			     __dlist_print_pos = __dlist_print_pos->next) {                               \
				type *entry = dlist_entry(__dlist_print_pos, type, member);               \
				klog_debug("dlist_print: " field_fmt, entry->field);                      \
				__dlist_print_len++;                                                      \
				if (__dlist_print_len > max_len) {                                        \
					klog_debug("dlist_print: exceeded length %d", __dlist_print_len); \
					break;                                                            \
				}                                                                         \
			}                                                                                 \
		}                                                                                         \
	} while (0)
#endif // __KERNEL__

#endif /* DLIST_H */
