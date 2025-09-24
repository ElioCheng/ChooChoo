#ifndef KLOG_H
#define KLOG_H

#include "types.h"

// Log levels
#define KLOG_NONE 0
#define KLOG_PANIC 1
#define KLOG_ERROR 2
#define KLOG_WARNING 3
#define KLOG_INFO 4
#define KLOG_DEBUG 5

// Default to INFO for release builds, DEBUG for debug builds
#ifndef KLOG_COMPILE_LEVEL
#ifdef RELEASE_BUILD
#define KLOG_COMPILE_LEVEL KLOG_INFO
#else
#define KLOG_COMPILE_LEVEL KLOG_DEBUG
#endif
#endif

// Output destinations
#define KLOG_DEST_CONSOLE 1 << 0
#define KLOG_DEST_MEMORY 1 << 1

#define KLOG_LOCATION_SIZE 128
#define KLOG_BUF_SIZE 512
#define KLOG_MAX_ENTRIES 1024

// Log entry
struct klog_entry {
	u32 timestamp;
	u8 level;
	u8 cpu;
	char location[KLOG_LOCATION_SIZE];
	char message[KLOG_BUF_SIZE];
};

void klog_init(u32 destinations);
void klog_set_destinations(u32 dest);
u32 klog_get_destinations(void);
int klog_read_all_unread(struct klog_entry *entries, size_t max_entries, size_t *num_entries);
int klog_read_all_unread_formatted(char *formatted_logs, size_t size, size_t *num_entries);
int klog_read_range_formatted(char **formatted_logs, size_t size, int start_idx, int end_idx, size_t *num_entries);
void klog_print_all_unread(void);
void klog_internal(u8 level, const char *func, const char *line, const char *fmt, ...);
void klog_force(u8 level, const char *func, const char *line, const char *fmt, ...);

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Optimized macros that avoid evaluating format expressions when log level is disabled
#define klog_panic(fmt, ...)                                                                         \
	do {                                                                                         \
		if (KLOG_PANIC <= KLOG_COMPILE_LEVEL) {                                              \
			klog_internal(KLOG_PANIC, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__); \
		}                                                                                    \
	} while (0)

#define klog_error(fmt, ...)                                                                         \
	do {                                                                                         \
		if (KLOG_ERROR <= KLOG_COMPILE_LEVEL) {                                              \
			klog_internal(KLOG_ERROR, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__); \
		}                                                                                    \
	} while (0)

#define klog_warning(fmt, ...)                                                                         \
	do {                                                                                           \
		if (KLOG_WARNING <= KLOG_COMPILE_LEVEL) {                                              \
			klog_internal(KLOG_WARNING, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__); \
		}                                                                                      \
	} while (0)

#define klog_info(fmt, ...)                                                                         \
	do {                                                                                        \
		if (KLOG_INFO <= KLOG_COMPILE_LEVEL) {                                              \
			klog_internal(KLOG_INFO, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__); \
		}                                                                                   \
	} while (0)

#define klog_debug(fmt, ...)                                                                         \
	do {                                                                                         \
		if (KLOG_DEBUG <= KLOG_COMPILE_LEVEL) {                                              \
			klog_internal(KLOG_DEBUG, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__); \
		}                                                                                    \
	} while (0)

#define klog_force_panic(fmt, ...) klog_force(KLOG_PANIC, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__)
#define klog_force_error(fmt, ...) klog_force(KLOG_ERROR, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__)
#define klog_force_warning(fmt, ...) klog_force(KLOG_WARNING, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__)
#define klog_force_info(fmt, ...) klog_force(KLOG_INFO, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__)
#define klog_force_debug(fmt, ...) klog_force(KLOG_DEBUG, __func__, TOSTRING(__LINE__), fmt, ##__VA_ARGS__)

#endif /* klog.h */
