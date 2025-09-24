#ifndef SYMBOL_H
#define SYMBOL_H

#include <stdint.h>

/**
 * Structure representing a kernel symbol
 */
typedef struct {
	uint64_t addr; // Symbol address
	char *name; // Symbol name
} kernel_symbol_t;

/**
 * Look up a symbol name by its address
 * @param addr Address to look up
 * @return Symbol name or "unknown" if not found
 */
const char *symbol_lookup(uint64_t addr);

/**
 * Initialize the symbol table
 */
void symbol_init(void);

#endif /* SYMBOL_H */
