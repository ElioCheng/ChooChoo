#include "arch/cpu.h"
#include "arch/rpi.h"
#include "printf.h"
#include "types.h"
#include "timer/time.h"
#include "klog.h"
#include "string.h"
#include "uart.h"
#include <stddef.h>

// Global variables
static struct klog_entry klog_buffer[KLOG_MAX_ENTRIES];
static u32 klog_write_index = 0;
static u32 klog_read_index = 0;
static u32 klog_count = 0;
static u32 klog_destinations = KLOG_DEST_CONSOLE | KLOG_DEST_MEMORY;

// Initialize the logging system
void klog_init(u32 destinations)
{
	// Initialize the log buffer
	for (int i = 0; i < KLOG_MAX_ENTRIES; i++) {
		klog_buffer[i].timestamp = 0;
		klog_buffer[i].level = 0;
		klog_buffer[i].message[0] = '\0';
	}

	klog_set_destinations(destinations);
}

// Set output destinations
void klog_set_destinations(u32 dest)
{
	klog_destinations = dest;
}

u32 klog_get_destinations(void)
{
	return klog_destinations;
}

// Get current timestamp in milliseconds
static u32 klog_get_timestamp(void)
{
	return time_get_boot_time_tick();
}

static size_t klog_format_entry(struct klog_entry *entry, char *buf, size_t size)
{
	const char *level_str;
	switch (entry->level) {
	case KLOG_PANIC:
		level_str = "PANIC";
		break;
	case KLOG_ERROR:
		level_str = "ERROR";
		break;
	case KLOG_WARNING:
		level_str = "WARNING";
		break;
	case KLOG_INFO:
		level_str = "INFO";
		break;
	case KLOG_DEBUG:
		level_str = "DEBUG";
		break;
	default:
		level_str = "UNKNOWN";
		break;
	}

	char time_str[TIME_STYLE_SSMS_BUF_SIZE];
	time_format_time(time_str, entry->timestamp, TIME_STYLE_SSMS);
	int ret = snprintf(buf, size, "[%s][%d][%s][%s] %s\r\n", time_str, entry->cpu, level_str, entry->location,
			   entry->message);
	return ret;
}

static void klog_write_entry(u8 level, const char *func, const char *line, const char *fmt, va_list args)
{
	u8 cpu = get_cpu_id();
	char buf[KLOG_BUF_SIZE];
	char location[KLOG_LOCATION_SIZE];

	__raw_vsnprintf(buf, KLOG_BUF_SIZE, fmt, &args);
	snprintf(location, KLOG_LOCATION_SIZE, "%s:%s", func, line);

	u32 timestamp = klog_get_timestamp();
	struct klog_entry *entry = &klog_buffer[klog_write_index];
	entry->timestamp = timestamp;
	entry->level = level;
	strncpy(entry->message, buf, sizeof(entry->message) - 1);
	strncpy(entry->location, location, sizeof(entry->location) - 1);
	entry->message[sizeof(entry->message) - 1] = '\0';
	entry->location[sizeof(entry->location) - 1] = '\0';
	entry->cpu = cpu;

	if (klog_destinations & KLOG_DEST_MEMORY) {
		klog_write_index = (klog_write_index + 1) % KLOG_MAX_ENTRIES;
		if (klog_count < KLOG_MAX_ENTRIES) {
			klog_count++;
		} else {
			klog_read_index = (klog_read_index + 1) % KLOG_MAX_ENTRIES;
		}
	}

	if (klog_destinations & KLOG_DEST_CONSOLE) {
		klog_format_entry(entry, buf, KLOG_BUF_SIZE);
		uart_printf(CONSOLE, KLOG_BUF_SIZE, buf);
	}
}

void klog_internal(u8 level, const char *func, const char *line, const char *fmt, ...)
{
	if (level > KLOG_COMPILE_LEVEL) {
		return;
	}

	va_list args;
	va_start(args, fmt);
	klog_write_entry(level, func, line, fmt, args);
	va_end(args);
}

void klog_force(u8 level, const char *func, const char *line, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	klog_write_entry(level, func, line, fmt, args);
	va_end(args);
}

int klog_read(struct klog_entry *entry)
{
	if (klog_count == 0 || !entry) {
		return -1;
	}

	memcpy(entry, &klog_buffer[klog_read_index], sizeof(struct klog_entry));
	klog_read_index = (klog_read_index + 1) % KLOG_MAX_ENTRIES;
	klog_count--;

	return 0;
}

int klog_read_formatted(char *buf, size_t size)
{
	struct klog_entry entry;

	if (klog_read(&entry) != 0) {
		return -1;
	}

	klog_format_entry(&entry, buf, size);

	return 0;
}

void klog_clear(void)
{
	klog_write_index = 0;
	klog_read_index = 0;
	klog_count = 0;
}

int klog_read_all_unread(struct klog_entry *entries, size_t max_entries, size_t *num_entries)
{
	if (!entries || !num_entries || max_entries == 0) {
		return -1;
	}

	if (klog_count == 0) {
		*num_entries = 0;
		return 0;
	}

	size_t count = 0;
	u32 current_read = klog_read_index;

	// Read entries until we either reach the write index or fill the buffer
	while (count < klog_count && count < max_entries && current_read != klog_write_index) {
		memcpy(&entries[count], &klog_buffer[current_read], sizeof(struct klog_entry));
		current_read = (current_read + 1) % KLOG_MAX_ENTRIES;
		count++;
	}

	klog_read_index = current_read;

	*num_entries = count;
	return 0;
}

struct klog_entry entries[1024];

int klog_read_all_unread_formatted(char *formatted_logs, size_t size, size_t *num_entries)
{
	if (klog_read_all_unread(entries, *num_entries, num_entries) != 0) {
		return 0;
	}

	if (*num_entries == 0) {
		return 0;
	}

	size_t total_len = 0;
	for (u32 i = 0; i < *num_entries; i++) {
		size_t len = klog_format_entry(&entries[i], formatted_logs + total_len, size - total_len);
		total_len += len;
	}

	return total_len;
}

void klog_print_all_unread(void)
{
	char klog_buffer[1024 * 100];
	size_t num_entries = KLOG_MAX_ENTRIES;
	int result = klog_read_all_unread_formatted(klog_buffer, sizeof(klog_buffer), &num_entries);
	if (result > 0) {
		uart_puts(CONSOLE, klog_buffer);
		uart_puts(CONSOLE, "\n\r");
	}
}

/**
 * Read log entries from a specific range and format them
 *
 * @param formatted_logs Array of char buffers to store formatted log entries
 * @param size Size of each buffer in formatted_logs
 * @param start_idx Starting index (can be negative for relative indexing from the latest entry)
 * @param end_idx Ending index (can be negative for relative indexing from the latest entry)
 * @param num_entries Pointer to store the number of entries read
 * @return 0 on success, -1 on failure
 *
 * For relative indexing, -1 refers to the latest entry, -2 to the second latest, and so on.
 * When start_idx and end_idx are both negative, the range is interpreted as the last abs(start_idx) entries.
 * For example, start_idx=-20, end_idx=0 will return the last 20 entries.
 */
int klog_read_range_formatted(char **formatted_logs, size_t size, int start_idx, int end_idx, size_t *num_entries)
{
	if (!formatted_logs || !num_entries || size == 0) {
		return -1;
	}

	if (klog_count == 0) {
		*num_entries = 0;
		return 0;
	}

	// Convert relative indices to absolute indices
	if (start_idx < 0) {
		start_idx = (int)klog_count + start_idx;
		if (start_idx < 0)
			start_idx = 0;
	}

	if (end_idx < 0) {
		end_idx = (int)klog_count + end_idx;
		if (end_idx < 0)
			end_idx = 0;
	}

	// Handle special case where end_idx is 0
	if (end_idx == 0 && start_idx < 0) {
		end_idx = klog_count - 1;
	}

	// Ensure indices are within bounds
	if (start_idx >= (int)klog_count)
		start_idx = klog_count - 1;
	if (end_idx >= (int)klog_count)
		end_idx = klog_count - 1;

	// Ensure start_idx <= end_idx
	if (start_idx > end_idx) {
		int temp = start_idx;
		start_idx = end_idx;
		end_idx = temp;
	}

	// Calculate how many entries to read
	size_t count = end_idx - start_idx + 1;
	if (count > *num_entries) {
		count = *num_entries;
	}

	// Calculate the actual buffer indices
	u32 start_buf_idx = (klog_write_index - klog_count + start_idx) % KLOG_MAX_ENTRIES;

	// Read the entries
	for (size_t i = 0; i < count; i++) {
		u32 buf_idx = (start_buf_idx + i) % KLOG_MAX_ENTRIES;
		klog_format_entry(&klog_buffer[buf_idx], formatted_logs[i], size);
	}

	*num_entries = count;
	return 0;
}
