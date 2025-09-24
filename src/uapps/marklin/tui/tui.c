#include "marklin/tui/tui.h"
#include "marklin/msgqueue/api.h"
#include "marklin/conductor/api.h"
#include "marklin/conductor/sensor.h"
#include "clock.h"
#include "marklin/train2/train.h"
#include "stdbool.h"
#include "syscall.h"
#include "types.h"
#include "klog.h"
#include "string.h"
#include "printf.h"
#include "marklin/controller/marklin.h"
#include "marklin/controller/api.h"
#include "io.h"
#include "name.h"
#include <stdarg.h>

#define LOG_MODULE "TUI"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

static tui_state_t tui_state;

// Function prototypes for internal functions
static void tui_update_track_panel(void);
void tui_record_sensor_trigger(u8 bank, u8 sensor_num);
static void tui_console_output(const char *msg);
static void tui_process_shell_command(void);
static void tui_process_command(const char *command);
static void tui_process_block_reservation_update(const marklin_msgqueue_message_t *message);
static void tui_init_block_status(void);
static void tui_display_block_reservations(void);

// Frame buffer functions
static void frame_buffer_init(void);
static void frame_buffer_append(const char *data);
static void frame_buffer_flush(void);
static void frame_buffer_printf(const char *format, ...);

static int parse_int(const char *str, int *pos);
static char parse_char(const char *str, int *pos);

// Buffer for each panel
#define STATUS_BUFFER_SIZE 512
#define INPUT_BUFFER_SIZE 256
#define TRACK_BUFFER_SIZE 2048

static char status_buffer[STATUS_BUFFER_SIZE];
static char input_buffer[INPUT_BUFFER_SIZE];
static char track_buffer[TRACK_BUFFER_SIZE];

// Buffers to track last drawn state
static char status_last_buffer[STATUS_BUFFER_SIZE];
static char input_last_buffer[INPUT_BUFFER_SIZE];
static char track_last_buffer[TRACK_BUFFER_SIZE];

// Frame buffer for batched terminal output
#define FRAME_BUFFER_SIZE 8192
static char frame_buffer[FRAME_BUFFER_SIZE];
static u32 frame_buffer_pos = 0;

// ANSI escape sequences for terminal control
#define CLEAR_SCREEN "\033[2J"
#define CURSOR_HOME "\033[H"
#define CURSOR_POSITION "\033[%d;%dH"
#define SAVE_CURSOR "\033[s"
#define RESTORE_CURSOR "\033[u"
#define HIDE_CURSOR "\033[?25l"
#define SHOW_CURSOR "\033[?25h"
#define SET_SCROLL_REGION "\033[%d;%dr"
#define RESET_SCROLL_REGION "\033[r"

// Refresh rate control
#define TUI_MIN_UPDATE_INTERVAL_MS 100
#define TUI_BYTES_PER_SECOND 11000

// Track panel variables
#define MAX_RECENT_SENSORS 10
typedef struct {
	u8 bank;
	u8 sensor_num;
	u64 last_trigger_tick;
} recent_sensor_t;

static recent_sensor_t recent_sensors[MAX_RECENT_SENSORS];
static u8 recent_sensor_index = 0;

static int clock_server_tid = -1;
static marklin_msgqueue_subscription_t sensor_subscription;
static int sensor_subscription_active = 0;
static marklin_msgqueue_subscription_t block_subscription;
static int block_subscription_active = 0;
static u32 console_output_start_y = 0; // Y position where console output starts
static u32 console_output_current_y = 0; // Current Y position for next console output
static u64 last_status_update_time = 0; // Last time status was updated
static char cached_time_str[32] = { 0 }; // Cached time string
static u8 track_panel_needs_update = 1; // Flag to track when track panel needs updating

// Block reservation tracking
#define MAX_BLOCKS 30
typedef struct {
	u32 block_id;
	u8 owner_train_id;
	block_reservation_status_t status;
	u64 last_update_time;
	char entry_sensor_name[16];
} tui_block_status_t;

static tui_block_status_t block_status[MAX_BLOCKS];
static u8 block_status_initialized = 0;

// Initialize block status array
static void tui_init_block_status(void)
{
	if (block_status_initialized) {
		return;
	}

	for (int i = 0; i < MAX_BLOCKS; i++) {
		block_status[i].block_id = i;
		block_status[i].owner_train_id = 0;
		block_status[i].status = BLOCK_STATUS_FREE;
		block_status[i].last_update_time = 0;
		block_status[i].entry_sensor_name[0] = '\0';
	}

	block_status_initialized = 1;
}

// Function to process block reservation update messages
static void tui_process_block_reservation_update(const marklin_msgqueue_message_t *message)
{
	if (!message)
		return;

	marklin_block_reservation_data_t *block_update =
		MARKLIN_MSGQUEUE_CAST_TO(marklin_block_reservation_data_t, message);
	if (!block_update) {
		klog_error("TUI: Invalid block reservation message format (size: %u, expected: %u)", message->data_size,
			   (u32)sizeof(marklin_block_reservation_data_t));
		return;
	}

	// Validate block ID
	if (block_update->block_id >= MAX_BLOCKS) {
		klog_error("TUI: Invalid block ID: %u (max: %d)", block_update->block_id, MAX_BLOCKS - 1);
		return;
	}

	// Initialize block status if not done yet
	tui_init_block_status();

	// Update block status
	tui_block_status_t *block = &block_status[block_update->block_id];
	block->owner_train_id = block_update->owner_train_id;
	block->status = block_update->status;
	block->last_update_time = block_update->timestamp;

	// Copy entry sensor name
	strncpy(block->entry_sensor_name, block_update->entry_sensor_name, 15);
	block->entry_sensor_name[15] = '\0';

	// Mark track panel for update
	track_panel_needs_update = 1;
}

// Function to process sensor update messages
static void tui_process_sensor_update(const marklin_msgqueue_message_t *message)
{
	if (!message)
		return;

	marklin_sensor_state_t *sensor_update = MARKLIN_MSGQUEUE_CAST_TO(marklin_sensor_state_t, message);
	if (!sensor_update) {
		klog_error("TUI: Invalid sensor update message format (size: %u, expected: %u)", message->data_size,
			   (u32)sizeof(marklin_sensor_state_t));
		return;
	}

	// Validate sensor data ranges
	if (sensor_update->bank >= MARKLIN_SENSOR_BANK_COUNT || sensor_update->sensor_id == 0 ||
	    sensor_update->sensor_id > 16) {
		klog_error("TUI: Invalid sensor data - bank: %d, sensor_id: %d", sensor_update->bank,
			   sensor_update->sensor_id);
		return;
	}

	// Only record sensor triggers (when triggered = 1)
	if (sensor_update->triggered) {
		tui_record_sensor_trigger(sensor_update->bank, sensor_update->sensor_id);
		track_panel_needs_update = 1; // Mark track panel for update
	}
}

static inline void tui_setup_panels(u8 panel, u32 x, u32 y, u32 width, u32 height, u8 style, char *buffer,
				    u64 buffer_size, u64 buffer_pos, char *last_buffer, const char *title)
{
	tui_state.panels[panel].x = x;
	tui_state.panels[panel].y = y;
	tui_state.panels[panel].width = width;
	tui_state.panels[panel].height = height;
	tui_state.panels[panel].style = style;
	tui_state.panels[panel].buffer = buffer;
	tui_state.panels[panel].buffer_size = buffer_size;
	tui_state.panels[panel].buffer_pos = buffer_pos;
	tui_state.panels[panel].last_buffer = last_buffer;
	tui_state.panels[panel].last_buffer_pos = 0;
	tui_state.panels[panel].dirty = 1;
	tui_state.panels[panel].border_drawn = 0;
	strncpy(tui_state.panels[panel].title, title, 16);
}

void tui_init(void)
{
	clock_server_tid = WhoIs(CLOCK_SERVER_NAME);
	if (clock_server_tid < 0) {
		Panic("Clock server not found");
	}

	memset(&tui_state, 0, sizeof(tui_state));
	memset(&recent_sensors, 0, sizeof(recent_sensors));
	recent_sensor_index = 0;

	tui_state.tid = MyTid();

	// Subscribe to sensor updates
	marklin_error_t subscription_result =
		Marklin_MsgQueue_Subscribe(MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE, &sensor_subscription);
	if (subscription_result == MARKLIN_ERROR_OK) {
		sensor_subscription_active = 1;
		klog_info("TUI: Subscribed to sensor updates");
	} else {
		sensor_subscription_active = 0;
		klog_error("TUI: Failed to subscribe to sensor updates: %d", subscription_result);
	}

	// Subscribe to block reservation updates
	marklin_error_t block_subscription_result =
		Marklin_MsgQueue_Subscribe(MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION, &block_subscription);
	if (block_subscription_result == MARKLIN_ERROR_OK) {
		block_subscription_active = 1;
		klog_info("TUI: Subscribed to block reservation updates");
	} else {
		block_subscription_active = 0;
		klog_error("TUI: Failed to subscribe to block reservation updates: %d", block_subscription_result);
	}

	// Initialize block status tracking
	tui_init_block_status();

	// Clear buffers
	for (int i = 0; i < STATUS_BUFFER_SIZE; i++) {
		status_buffer[i] = 0;
		status_last_buffer[i] = 0;
	}
	for (int i = 0; i < INPUT_BUFFER_SIZE; i++) {
		input_buffer[i] = 0;
		input_last_buffer[i] = 0;
	}
	for (int i = 0; i < TRACK_BUFFER_SIZE; i++) {
		track_buffer[i] = 0;
		track_last_buffer[i] = 0;
	}

	// Status panel (top)
	tui_setup_panels(TUI_PANEL_STATUS, 0, 1, TUI_SCREEN_WIDTH, 3, TUI_STYLE_BORDER, status_buffer,
			 STATUS_BUFFER_SIZE, 0, status_last_buffer, "System Status");

	// Track panel (below status, sharing bottom border with status)
	tui_setup_panels(TUI_PANEL_TRACK, 0, 3, TUI_SCREEN_WIDTH, 18, TUI_STYLE_BORDER, track_buffer, TRACK_BUFFER_SIZE,
			 0, track_last_buffer, "Track Status");

	// Input panel (below track panel, sharing bottom border with track)
	tui_setup_panels(TUI_PANEL_INPUT, 0, 20, TUI_SCREEN_WIDTH, 3, TUI_STYLE_BORDER, input_buffer, INPUT_BUFFER_SIZE,
			 0, input_last_buffer, "Command Input");

	// Set console output start position (below input panel)
	console_output_start_y = 23;
	console_output_current_y = console_output_start_y;

	tui_state.input_pos = 0;
	tui_state.last_update_time_tick = 0;
	tui_state.active = 0;
	tui_state.shell_mode = 0;
	tui_state.f1_state = TUI_F1_STATE_NORMAL;
}

void tui_mark_panel_dirty(u8 panel_id)
{
	if (panel_id < TUI_PANEL_COUNT) {
		tui_state.panels[panel_id].dirty = 1;
	}
}

void tui_force_redraw(void)
{
	for (u8 i = 0; i < TUI_PANEL_COUNT; i++) {
		tui_state.panels[i].dirty = 1;
		tui_state.panels[i].border_drawn = 0;
	}
}

void tui_clear_buffer(u8 panel_id)
{
	tui_panel_t *panel = &tui_state.panels[panel_id];
	panel->buffer_pos = 0;
	panel->buffer[0] = '\0';
}

static u8 tui_full_redraw = 0;

void tui_start(void)
{
	tui_state.active = 1;

	tui_clear_screen();

	tui_clear_buffer(TUI_PANEL_INPUT);

	tui_force_redraw();

	tui_full_redraw = 1;

	tui_update_track_panel();

	tui_update_status();

	// Draw everything
	tui_draw();

	tui_full_redraw = 0;

	// Set scroll region below the panels
	console_printf(SET_SCROLL_REGION, console_output_start_y, TUI_SCREEN_HEIGHT);

	// Position cursor at console output area and update current position
	console_printf(CURSOR_POSITION, console_output_start_y, 1);
	console_output_current_y = console_output_start_y;

	// Display initial messages using tui_console_output to track position
	tui_console_output("");
	tui_console_output("TUI Interface Ready");
	tui_console_output("Type 'help' for commands or press F1 to toggle to shell mode");
	tui_console_output("");
	tui_console_output("*** SYSTEM INITIALIZATION REQUIRED ***");
	tui_console_output("Please type 'reset A' or 'reset B' to initialize the system.");
	tui_console_output("");
	if (sensor_subscription_active) {
		tui_console_output("Sensor updates: Connected via message queue");
	} else {
		tui_console_output("Sensor updates: Not available");
	}

	if (block_subscription_active) {
		tui_console_output("Block reservations: Connected via message queue");
	} else {
		tui_console_output("Block reservations: Not available");
	}
}

void tui_stop(void)
{
	tui_state.active = 0;

	// Unsubscribe from sensor updates
	if (sensor_subscription_active) {
		marklin_error_t unsubscribe_result = Marklin_MsgQueue_Unsubscribe(&sensor_subscription);
		if (unsubscribe_result == MARKLIN_ERROR_OK) {
			klog_info("TUI: Unsubscribed from sensor updates");
		} else {
			klog_error("TUI: Failed to unsubscribe from sensor updates: %d", unsubscribe_result);
		}
		sensor_subscription_active = 0;
	}

	// Unsubscribe from block reservation updates
	if (block_subscription_active) {
		marklin_error_t unsubscribe_result = Marklin_MsgQueue_Unsubscribe(&block_subscription);
		if (unsubscribe_result == MARKLIN_ERROR_OK) {
			klog_info("TUI: Unsubscribed from block reservation updates");
		} else {
			klog_error("TUI: Failed to unsubscribe from block reservation updates: %d", unsubscribe_result);
		}
		block_subscription_active = 0;
	}

	// Reset scroll region
	console_puts(RESET_SCROLL_REGION);

	tui_clear_screen();
}

void tui_toggle_mode(void)
{
	tui_state.shell_mode = !tui_state.shell_mode;

	if (tui_state.shell_mode) {
		// Entering shell mode - disable TUI display and idle display
		console_puts(RESET_SCROLL_REGION);
		console_puts(SHOW_CURSOR);
		tui_clear_screen();

		// Disable idle display to prevent CPU usage overlay in shell mode
		ToggleIdleDisplay();

		// Print shell mode message
		console_puts("Shell mode enabled. Press F1 to return to TUI mode.\r\n");
		console_puts("> ");
	} else {
		// Returning to TUI mode - restore TUI display and idle display
		console_puts(HIDE_CURSOR);
		tui_clear_screen();

		// Re-enable idle display for TUI mode
		ToggleIdleDisplay();

		// Force complete redraw of all panels including borders
		tui_force_redraw();
		tui_full_redraw = 1;
		tui_update_track_panel();
		tui_update_status();
		tui_draw();
		tui_full_redraw = 0;

		// Restore scroll region and position cursor
		console_printf(SET_SCROLL_REGION, console_output_start_y, TUI_SCREEN_HEIGHT);
		console_printf(CURSOR_POSITION, console_output_start_y, 1);
		console_output_current_y = console_output_start_y;

		// Clear input buffer
		tui_clear_buffer(TUI_PANEL_INPUT);
		tui_state.input_pos = 0;

		// Print return message
		tui_console_output("TUI mode restored. Press F1 to toggle to shell mode.");
	}
}

void tui_clear_screen(void)
{
	console_puts(CLEAR_SCREEN CURSOR_HOME);
}

void tui_set_cursor(u8 x, u8 y)
{
	console_printf(CURSOR_POSITION, y + 1, x + 1);
}

static void draw_box(u8 x, u8 y, u8 width, u8 height, const char *title, u8 style, u8 title_only, u8 panel_id)
{
	// Suppress unused parameter warnings
	(void)height;
	(void)title_only;
	(void)panel_id;
	(void)title;

	if (style != TUI_STYLE_BORDER) {
		return;
	}

	char line_buf[TUI_SCREEN_WIDTH * 4]; // Account for UTF-8 characters

	// Draw title separator line
	// if (title && *title) {
	// 	int title_len = strlen(title);
	// 	int separator_len = width - title_len - 4; // Account for "title: " and space

	// 	if (separator_len > 0) {
	// 		int pos = snprintf(line_buf, sizeof(line_buf), "%s ", title);
	// 		// Add separator characters with bounds checking
	// 		for (int i = 0; i < separator_len && pos < (int)(sizeof(line_buf) - 4); i++) {
	// 			pos += snprintf(&line_buf[pos], sizeof(line_buf) - pos, "─");
	// 		}
	// 	} else {
	// 		// If title is too long, just show title with minimal separator
	// 		snprintf(line_buf, sizeof(line_buf), "%s ──", title);
	// 	}
	// } else {
	// No title, just draw a horizontal line
	int pos = 0;
	for (int i = 0; i < width - 1 && pos < (int)(sizeof(line_buf) - 4); i++) {
		pos += snprintf(&line_buf[pos], sizeof(line_buf) - pos, "─");
	}
	// }

	frame_buffer_printf("\033[%d;%dH%s", y + 1, x + 1, line_buf);
}

static u8 is_line_changed(const char *buffer, const char *last_buffer, u32 line_start, u32 line_length)
{
	for (u32 i = 0; i < line_length; i++) {
		if (buffer[line_start + i] != last_buffer[line_start + i]) {
			return 1;
		}
	}
	return 0;
}

void tui_draw_panel(u8 panel_id)
{
	if (panel_id >= TUI_PANEL_COUNT) {
		return;
	}

	tui_panel_t *panel = &tui_state.panels[panel_id];

	// Add an indicator (*) to the title when panel is dirty
	char title[32];
	if (panel->dirty) {
		snprintf(title, 31, "%s*", panel->title);
	} else {
		snprintf(title, 31, "%s", panel->title);
	}

	// Only draw the box when panel is dirty or border hasn't been drawn yet
	if (panel->dirty || !panel->border_drawn) {
		draw_box(panel->x, panel->y, panel->width, panel->height, title, panel->style, 0, panel_id);
		panel->border_drawn = 1;
	}

	// Skip rest of drawing if panel is not dirty
	if (!panel->dirty && !tui_full_redraw) {
		return;
	}

	if (panel_id != TUI_PANEL_INPUT && !tui_full_redraw && panel->last_buffer_pos == panel->buffer_pos &&
	    memcmp(panel->last_buffer, panel->buffer, panel->buffer_pos) == 0) {
		panel->dirty = 0;
		return;
	}

	char line_buf[TUI_SCREEN_WIDTH + 1];
	char *buf_ptr = panel->buffer;
	char *last_buf_ptr = panel->last_buffer;
	u8 line = 0;
	u8 col = 0;
	u32 line_start = 0;
	// klog_info("panel %d: panel->buffer: %s, panel->last_buffer: %s", panel_id, panel->buffer, panel->last_buffer);

	for (u16 i = 0; i <= panel->buffer_pos && line < panel->height - 1; i++) {
		if (i == panel->buffer_pos || buf_ptr[i] == '\n') {
			// Check if this line is different from last time
			u8 line_changed = 1;

			if (i == panel->buffer_pos) {
				line_buf[col] = '\0';

				// If this is the end of buffer, check if the line has changed
				if (panel->last_buffer_pos >= line_start) {
					line_changed = is_line_changed(buf_ptr, last_buf_ptr, line_start, col);
				}
			} else if (buf_ptr[i] == '\n') {
				line_buf[col] = '\0';

				// Check if line has changed compared to last buffer
				if (panel->last_buffer_pos >= (line_start + col)) {
					line_changed = is_line_changed(buf_ptr, last_buf_ptr, line_start, col);
				}
			}

			// Force redraw during full redraw mode
			if (tui_full_redraw) {
				line_changed = 1;
			}

			// Only draw the line if it has changed
			if (line_changed) {
				// For input panel, add prompt ">" before the content
				if (panel_id == TUI_PANEL_INPUT && line == 0) {
					u32 clear_len = panel->width - 2 - col - 3; // Account for "> " prompt
					frame_buffer_printf("\033[%d;%dH> %s%*s", panel->y + 2 + line, panel->x + 2,
							    line_buf, clear_len, "");
				} else {
					// Combine cursor positioning, text output, and clearing into single frame buffer call
					u32 clear_len = panel->width - 2 - col - 1;
					frame_buffer_printf("\033[%d;%dH%s%*s", panel->y + 2 + line, panel->x + 2,
							    line_buf, clear_len, "");
				}
			}

			line++;
			line_start = i + 1;
			col = 0;
			if (i < panel->buffer_pos && buf_ptr[i] == '\n') {
				continue;
			}
		}

		if (i < panel->buffer_pos) {
			line_buf[col++] = buf_ptr[i];
		}
	}

	memcpy(panel->last_buffer, panel->buffer, panel->buffer_size);
	panel->last_buffer_pos = panel->buffer_pos;
	panel->dirty = 0;
}

void tui_draw(void)
{
	// Initialize frame buffer for batched output
	frame_buffer_init();

	// Add cursor hiding to frame buffer
	frame_buffer_append(HIDE_CURSOR);

	for (u8 i = 0; i < TUI_PANEL_COUNT; i++) {
		tui_draw_panel(i);
	}

	// Add cursor showing to frame buffer (commented for now)
	// frame_buffer_append(SHOW_CURSOR);

	// Flush all buffered output at once
	frame_buffer_flush();
}

void tui_panel_add_message(u8 panel_id, const char *msg)
{
	if (panel_id >= TUI_PANEL_COUNT || !msg) {
		return;
	}

	tui_panel_t *panel = &tui_state.panels[panel_id];
	u16 msg_len = strlen(msg);

	// Add the new message
	strcpy(&panel->buffer[panel->buffer_pos], msg);
	panel->buffer_pos += msg_len;
	panel->buffer[panel->buffer_pos++] = '\n';
	panel->buffer[panel->buffer_pos] = '\0';
}

// Add text at a specific position in a panel
void tui_panel_add_text(u8 panel_id, u8 x, u8 y, const char *msg)
{
	if (panel_id >= TUI_PANEL_COUNT || !msg) {
		return;
	}

	tui_panel_t *panel = &tui_state.panels[panel_id];

	if (x >= panel->width - 2 || y >= panel->height - 2) {
		return;
	}

	// Combine cursor positioning and text output into single IPC call
	console_printf("\033[%d;%dH%s", panel->y + 2 + y, panel->x + 2 + x, msg);

	// Update the internal buffer to reflect the change
	u32 line_length = panel->width - 2;
	u32 line_start = (y * line_length) + x;
	u16 msg_len = strlen(msg);

	// Ensure buffer space
	if (line_start + msg_len < panel->buffer_size) {
		strcpy(&panel->buffer[line_start], msg);
	}
}

void tui_update_status(void)
{
	u64 current_time = Time(clock_server_tid);

	// Only update status every second (1000ms) or if cached string is empty
	if (cached_time_str[0] == 0 || (current_time - last_status_update_time) >= MS_TO_TICK(100)) {
		char time_str[32];
		time_format_time(time_str, current_time, TIME_STYLE_HHMMSSMS);

		// Only update if time string actually changed
		if (strcmp(time_str, cached_time_str) != 0) {
			strcpy(cached_time_str, time_str);
			last_status_update_time = current_time;

			tui_panel_t *panel = &tui_state.panels[TUI_PANEL_STATUS];
			panel->buffer_pos = 0;

			char status_line[TUI_SCREEN_WIDTH] = { 0 };
			snprintf(status_line, TUI_SCREEN_WIDTH, "ChooChoo OS | Uptime: %s | TUI TID: %d", time_str,
				 tui_state.tid);

			tui_panel_add_message(TUI_PANEL_STATUS, status_line);
			tui_mark_panel_dirty(TUI_PANEL_STATUS);
		}
	}
}

void tui_show_help(void)
{
	tui_console_output("Available commands:");
	tui_console_output("  help - Display this help");
	tui_console_output("  clear - Clear the console output");
	tui_console_output("");
	tui_console_output("Mode Management:");
	tui_console_output("  mode <train> <manual|waypoint> - Set operating mode");
	tui_console_output("");
	tui_console_output("Manual Mode Commands:");
	tui_console_output("  tr <train> <speed> - Set effective speed");
	tui_console_output("  rv <train> - Reverse train");
	tui_console_output("  hl <train> - Toggle headlight");
	tui_console_output("  stop <train> - Stop train");
	tui_console_output("");
	tui_console_output("Waypoint Mode Commands:");
	tui_console_output("  speed <train> <speed> - Set requested speed");
	tui_console_output("  dest <train> <sensor> [offset_mm] - Set destination");
	tui_console_output("  random <train> <on|off> - Enable/disable random destinations");
	tui_console_output("  estop <train> - Emergency stop");
	tui_console_output("");
	tui_console_output("System Commands:");
	tui_console_output("  sw <switch> <S/C> - Set switch direction");
	tui_console_output("  spawn <train> <sensor> [reverse] - Spawn train at sensor");
	tui_console_output("  reset <A/B> - Reset the track with type A or B");
	tui_console_output("  allsw <S/C> - Set all single switches");
	tui_console_output("  blocks - Display current block reservations");
	tui_console_output("  go - Start the demo function");
	tui_console_output("  q - Quit and reboot");
	tui_console_output("");
	tui_console_output("Offline Experiment Commands:");
	tui_console_output("  offexp <train> <type> <speeds...> - Start offline experiment");
	tui_console_output("    Types: vel, accel, stop");
	tui_console_output("    Examples: offexp 24 vel 12 13 14");
	tui_console_output("              offexp 24 accel 5 10 10 14 (pairs: 5->10, 10->14)");
	tui_console_output("  offstop <train> - Stop offline experiment");
	tui_console_output("  model <train> - Display kinematic model");
	tui_console_output("  debug <train> - Print comprehensive train debug info");
	tui_console_output("  clear <train> - Clear destination and reset to idle");
	tui_console_output("");
	tui_console_output("Interface Commands:");
	tui_console_output("  F1 - Toggle between TUI and shell mode");
}

void tui_process_input(char c)
{
	// Handle F1 key sequence detection first
	switch (tui_state.f1_state) {
	case TUI_F1_STATE_NORMAL:
		if (c == TUI_KEY_ESCAPE) {
			tui_state.f1_state = TUI_F1_STATE_ESC;
			return;
		}
		break;
	case TUI_F1_STATE_ESC:
		if (c == 'O') {
			tui_state.f1_state = TUI_F1_STATE_O;
			return;
		} else if (c == '[') {
			tui_state.f1_state = TUI_F1_STATE_BRACKET;
			return;
		} else {
			// Not F1 sequence, reset and process ESC + this character
			tui_state.f1_state = TUI_F1_STATE_NORMAL;
			// Process the ESC character first (if needed)
			// Then fall through to process current character
		}
		break;
	case TUI_F1_STATE_O:
		if (c == 'P') {
			// Complete F1 sequence: ESC O P
			tui_state.f1_state = TUI_F1_STATE_NORMAL;
			tui_toggle_mode();
			return;
		} else {
			// Not F1 sequence, reset
			tui_state.f1_state = TUI_F1_STATE_NORMAL;
		}
		break;
	case TUI_F1_STATE_BRACKET:
		if (c == '1') {
			// Could be F1 sequence: ESC [ 1 ~, but we'll handle basic ESC O P for now
			tui_state.f1_state = TUI_F1_STATE_NORMAL;
		} else {
			// Not F1 sequence, reset
			tui_state.f1_state = TUI_F1_STATE_NORMAL;
		}
		break;
	}

	// In shell mode, just echo the character and process commands on enter
	if (tui_state.shell_mode) {
		if (c == TUI_KEY_ENTER) {
			console_puts("\r\n");
			tui_process_shell_command();
			tui_state.input_pos = 0;
			console_puts("> ");
		} else if (c == TUI_KEY_BACKSPACE) {
			if (tui_state.input_pos > 0) {
				tui_state.input_pos--;
				console_puts("\b \b"); // Backspace, space, backspace
			}
		} else if (c >= 32 && c <= 126) { // Printable characters
			if (tui_state.input_pos < 127) {
				tui_state.input_buffer[tui_state.input_pos++] = c;
				console_printf("%c", c);
			}
		}
		return;
	}

	// Normal TUI mode processing
	tui_panel_t *panel = &tui_state.panels[TUI_PANEL_INPUT];

	if (c == TUI_KEY_ENTER) {
		panel->buffer[tui_state.input_pos] = '\0';

		char cmd_echo[INPUT_BUFFER_SIZE + 16];
		strcpy(cmd_echo, "> ");
		strncat(cmd_echo, panel->buffer, tui_state.input_pos);

		tui_console_output(cmd_echo);

		// Use shared command processing function
		tui_process_command(panel->buffer);

		// Clear input
		tui_state.input_pos = 0;
		tui_state.input_buffer[0] = '\0';
		tui_panel_t *panel = &tui_state.panels[TUI_PANEL_INPUT];
		panel->buffer_pos = 0;
		panel->buffer[0] = '\0';
	} else if (c == TUI_KEY_BACKSPACE) {
		// Handle backspace in TUI mode
		if (tui_state.input_pos > 0) {
			tui_state.input_pos--;
			tui_state.input_buffer[tui_state.input_pos] = '\0';
			panel->buffer_pos = tui_state.input_pos;
			panel->buffer[panel->buffer_pos] = '\0';
			tui_mark_panel_dirty(TUI_PANEL_INPUT);
		}
	} else if (c >= 32 && c <= 126) { // Printable characters
		// Handle regular character input in TUI mode
		if (tui_state.input_pos < 127) {
			tui_state.input_buffer[tui_state.input_pos] = c;
			panel->buffer[panel->buffer_pos] = c;
			tui_state.input_pos++;
			panel->buffer_pos++;
			panel->buffer[panel->buffer_pos] = '\0';
			tui_mark_panel_dirty(TUI_PANEL_INPUT);
		}
	}
}

static void tui_process_command(const char *command)
{
	if (strcmp(command, "help") == 0) {
		tui_show_help();
	} else if (strcmp(command, "clear") == 0) {
		if (tui_state.shell_mode) {
			console_puts(CLEAR_SCREEN CURSOR_HOME);
		} else {
			console_printf(CURSOR_POSITION "\033[J", console_output_start_y, 1);
			console_output_current_y = console_output_start_y;
		}
	} else if (strncmp(command, "tr ", 3) == 0) {
		// Train speed command: tr <train> <speed>
		int train_num = 0;
		int speed = 0;
		int pos = 3;
		train_num = parse_int(command, &pos);
		speed = parse_int(command, &pos);
		if (train_num > 0 && speed >= 0) {
			if (speed < 0 || speed > MARKLIN_TRAIN_MAX_SPEED) {
				tui_console_output("Invalid speed (0-14)");
			} else {
				char msg[64];
				marklin_train_command_t cmd;
				cmd.command_type = MARKLIN_TRAIN_CMD_MANUAL_SET_EFFECTIVE_SPEED;
				cmd.manual_set_effective_speed.effective_speed = speed;
				cmd.manual_set_effective_speed.headlight = MARKLIN_TRAIN_HEADLIGHT_AUTO;
				marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
				if (result == MARKLIN_ERROR_OK) {
					snprintf(msg, 64, "Set train %d effective speed to %d", train_num, speed);
				} else {
					snprintf(msg, 64, "Failed to set train %d speed: error %d", train_num, result);
				}
				tui_console_output(msg);
			}
		} else {
			tui_console_output("Usage: tr <train number> <speed>");
		}
	} else if (strncmp(command, "reset ", 6) == 0) {
		// Reset track command with track type: reset <A/B>
		int pos = 6;
		char track_type = parse_char(command, &pos);
		if (track_type == 'A' || track_type == 'a' || track_type == 'B' || track_type == 'b') {
			marklin_track_type_t type = (track_type == 'A' || track_type == 'a') ? MARKLIN_TRACK_TYPE_A :
											       MARKLIN_TRACK_TYPE_B;
			char *type_str = (type == MARKLIN_TRACK_TYPE_A) ? "A" : "B";
			char msg[64];
			snprintf(msg, 64, "Resetting system with track type %s", type_str);
			tui_console_output(msg);
			marklin_error_t result = Marklin_ControllerSystemReset(type);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "System reset complete with track type %s", type_str);
				tui_console_output(msg);
			} else {
				snprintf(msg, 64, "Failed to reset system: error %d", result);
				tui_console_output(msg);
			}
		} else {
			tui_console_output("Usage: reset <A/B>");
		}
	} else if (strncmp(command, "allsw ", 6) == 0) {
		// Set all switches to the same position
		int pos = 6;
		char direction = parse_char(command, &pos);
		if (direction == 'S' || direction == 's' || direction == 'C' || direction == 'c') {
			track_direction dir = (direction == 'S' || direction == 's') ? DIR_STRAIGHT : DIR_CURVED;
			char *dir_str = (dir == DIR_STRAIGHT) ? "straight" : "curved";
			char msg[64];
			snprintf(msg, 64, "Setting all switches to %s", dir_str);
			tui_console_output(msg);
			marklin_error_t result = Marklin_ControllerSetAllSwitches(dir);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "All switches set to %s", dir_str);
				tui_console_output(msg);
			} else {
				snprintf(msg, 64, "Failed to set switches: error %d", result);
				tui_console_output(msg);
			}
		} else {
			tui_console_output("Usage: allsw <S/C>");
		}
	} else if (strncmp(command, "rv ", 3) == 0) {
		// Train reverse command: rv <train>
		int train_num = 0;
		int pos = 3;
		train_num = parse_int(command, &pos);
		if (train_num > 0) {
			char msg[64];
			marklin_train_command_t cmd;
			cmd.command_type = MARKLIN_TRAIN_CMD_MANUAL_REVERSE;
			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Reversed train %d", train_num);
			} else {
				snprintf(msg, 64, "Failed to reverse train %d: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: rv <train number>");
		}
	} else if (strncmp(command, "sw ", 3) == 0) {
		// Switch command: sw <switch> <S/C>
		int switch_num = 0;
		int pos = 3;
		switch_num = parse_int(command, &pos);
		char direction = parse_char(command, &pos);
		if (switch_num > 0 && (direction == 'S' || direction == 's' || direction == 'C' || direction == 'c')) {
			char msg[64];
			track_direction dir = (direction == 'S' || direction == 's') ? DIR_STRAIGHT : DIR_CURVED;
			char *dir_str = (dir == DIR_STRAIGHT) ? "straight" : "curved";
			marklin_error_t result = Marklin_SetSwitch(switch_num, dir, 1, false);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Set switch %d to %s", switch_num, dir_str);
			} else {
				snprintf(msg, 64, "Failed to set switch %d: error %d", switch_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: sw <switch number> <S/C>");
		}
	} else if (strncmp(command, "hl ", 3) == 0) {
		// Headlight command: hl <train>
		int train_num = 0;
		int pos = 3;

		train_num = parse_int(command, &pos);

		if (train_num > 0) {
			char msg[64];

			marklin_train_command_t command;
			command.command_type = MARKLIN_TRAIN_CMD_MANUAL_TOGGLE_HEADLIGHT;

			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Toggled train %d headlight", train_num);
			} else {
				snprintf(msg, 64, "Failed to toggle train %d headlight: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: hl <train number>");
		}
	} else if (strncmp(command, "spawn ", 6) == 0) {
		// Spawn command: spawn <train> <sensor> [reverse]
		int train_num = 0;
		int pos = 6;
		char sensor_name[16] = { 0 };
		bool spawn_in_reverse = false;

		train_num = parse_int(command, &pos);

		// Skip whitespace
		while (command[pos] == ' ' || command[pos] == '\t') {
			pos++;
		}

		// Parse sensor name (like A1, B5, etc.)
		int sensor_pos = 0;
		while (command[pos] != '\0' && command[pos] != ' ' && command[pos] != '\t' && sensor_pos < 15) {
			sensor_name[sensor_pos++] = command[pos++];
		}
		sensor_name[sensor_pos] = '\0';

		// Skip whitespace after sensor name
		while (command[pos] == ' ' || command[pos] == '\t') {
			pos++;
		}

		// Check for optional reverse parameter
		if (command[pos] != '\0') {
			char reverse_param[16] = { 0 };
			int reverse_pos = 0;
			while (command[pos] != '\0' && command[pos] != ' ' && command[pos] != '\t' &&
			       reverse_pos < 15) {
				reverse_param[reverse_pos++] = command[pos++];
			}
			reverse_param[reverse_pos] = '\0';

			// Check if it's "reverse" or "r" (case-insensitive)
			if (strcmp(reverse_param, "reverse") == 0 || strcmp(reverse_param, "r") == 0 ||
			    strcmp(reverse_param, "REVERSE") == 0 || strcmp(reverse_param, "R") == 0) {
				spawn_in_reverse = true;
			} else {
				tui_console_output("Usage: spawn <train number> <sensor name> [reverse]");
				return;
			}
		}

		if (train_num > 0 && strlen(sensor_name) > 0) {
			char msg[64];
			int train_task_tid;

			marklin_error_t result =
				Marklin_ControllerSpawnTrainBySensor(train_num, sensor_name, &train_task_tid);
			if (result == MARKLIN_ERROR_OK) {
				if (spawn_in_reverse) {
					marklin_train_command_t reverse_cmd;
					reverse_cmd.command_type = MARKLIN_TRAIN_CMD_MANUAL_REVERSE;
					marklin_error_t reverse_result =
						Marklin_ControllerTrainCommand(train_num, &reverse_cmd);

					if (reverse_result == MARKLIN_ERROR_OK) {
						snprintf(msg, 64, "Spawned train %d at sensor %s in reverse (task: %d)",
							 train_num, sensor_name, train_task_tid);
					} else {
						snprintf(msg, 64,
							 "Spawned train %d at %s but failed to reverse: error %d",
							 train_num, sensor_name, reverse_result);
					}
				} else {
					snprintf(msg, 64, "Spawned train %d at sensor %s (task: %d)", train_num,
						 sensor_name, train_task_tid);
				}
			} else if (result == MARKLIN_ERROR_NOT_FOUND) {
				snprintf(msg, 64, "Sensor %s not found", sensor_name);
			} else {
				snprintf(msg, 64, "Failed to spawn train %d at %s: error %d", train_num, sensor_name,
					 result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: spawn <train number> <sensor name> [reverse]");
		}
	} else if (strncmp(command, "dest ", 5) == 0) {
		// Destination command: dest <train> <sensor> [offset_mm]
		int pos = 5;
		int train_num = parse_int(command, &pos);
		char msg[64];

		if (train_num > 0 && command[pos] == ' ') {
			pos++; // Skip space

			// Parse destination sensor name
			char destination_name[16] = { 0 };
			int dest_pos = 0;
			while (command[pos] != '\0' && command[pos] != ' ' && dest_pos < 15) {
				destination_name[dest_pos] = command[pos];
				dest_pos++;
				pos++;
			}
			destination_name[dest_pos] = '\0';

			if (strlen(destination_name) > 0) {
				// Check if there's an offset parameter
				int offset_mm = 0; // Default to 0 if not specified

				if (command[pos] == ' ') {
					pos++; // Skip space
					// Parse offset in millimeters
					offset_mm = parse_int(command, &pos);
				}

				// Validate offset range (-1000 to +1000 mm)
				if (offset_mm >= -1000 && offset_mm <= 1000) {
					// Create the train command
					marklin_train_command_t command;
					command.command_type = MARKLIN_TRAIN_CMD_SET_DESTINATION;
					strncpy(command.set_destination.destination_name, destination_name, 15);
					command.set_destination.destination_name[15] = '\0';
					command.set_destination.destination = NULL; // Will be resolved by train task
					command.set_destination.offset_mm = (kinematic_distance_t)offset_mm;

					marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
					if (result == MARKLIN_ERROR_OK) {
						if (offset_mm != 0) {
							snprintf(msg, 64, "Set train %d dest to %s offset %dmm",
								 train_num, destination_name, offset_mm);
						} else {
							snprintf(msg, 64, "Set train %d dest to %s", train_num,
								 destination_name);
						}
					} else {
						snprintf(msg, 64, "Failed to set destination for train %d: error %d",
							 train_num, result);
					}
				} else {
					snprintf(msg, 64, "Offset must be between -1000 and +1000 mm");
				}
			} else {
				snprintf(msg, 64, "Invalid destination name");
			}
		} else {
			snprintf(msg, 64, "Usage: dest <train number> <sensor name> [offset_mm]");
		}
		tui_console_output(msg);
	} else if (strncmp(command, "mode ", 5) == 0) {
		// Mode command: mode <train> <manual|waypoint>
		int pos = 5;
		int train_num = parse_int(command, &pos);
		char msg[64];

		if (train_num > 0 && command[pos] == ' ') {
			pos++; // Skip space

			// Parse mode name
			char mode_name[16] = { 0 };
			int mode_pos = 0;
			while (command[pos] != '\0' && command[pos] != ' ' && mode_pos < 15) {
				mode_name[mode_pos] = command[pos];
				mode_pos++;
				pos++;
			}
			mode_name[mode_pos] = '\0';

			// Convert mode name to enum
			train_operating_mode_t mode;
			bool valid_mode = true;
			if (strcmp(mode_name, "manual") == 0) {
				mode = TRAIN_MODE_MANUAL;
			} else if (strcmp(mode_name, "waypoint") == 0) {
				mode = TRAIN_MODE_WAYPOINT;
			} else {
				valid_mode = false;
			}

			if (valid_mode) {
				marklin_train_command_t command;
				command.command_type = MARKLIN_TRAIN_CMD_SET_MODE;
				command.set_mode.mode = mode;

				marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
				if (result == MARKLIN_ERROR_OK) {
					snprintf(msg, 64, "Set train %d to %s mode", train_num, mode_name);
				} else {
					snprintf(msg, 64, "Failed to set train %d mode: error %d", train_num, result);
				}
			} else {
				snprintf(msg, 64, "Invalid mode '%s'. Use manual or waypoint", mode_name);
			}
		} else {
			snprintf(msg, 64, "Usage: mode <train number> <manual|waypoint>");
		}
		tui_console_output(msg);
	} else if (strncmp(command, "speed ", 6) == 0) {
		// Speed command: speed <train> <speed>
		int train_num = 0;
		int speed = 0;
		int pos = 6;

		train_num = parse_int(command, &pos);
		speed = parse_int(command, &pos);

		if (train_num > 0 && speed >= 0) {
			if (speed < 0 || speed > MARKLIN_TRAIN_MAX_SPEED) {
				tui_console_output("Invalid speed (0-14)");
			} else {
				char msg[64];

				marklin_train_command_t command;
				command.command_type = MARKLIN_TRAIN_CMD_SET_REQUESTED_SPEED;
				command.set_requested_speed.requested_speed = speed;

				marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
				if (result == MARKLIN_ERROR_OK) {
					snprintf(msg, 64, "Set train %d requested speed to %d", train_num, speed);
				} else {
					snprintf(msg, 64, "Failed to set train %d requested speed: error %d", train_num,
						 result);
				}
				tui_console_output(msg);
			}
		} else {
			tui_console_output("Usage: speed <train number> <speed>");
		}
	} else if (strncmp(command, "stop ", 5) == 0) {
		// Stop command: stop <train>
		int train_num = 0;
		int pos = 5;

		train_num = parse_int(command, &pos);

		if (train_num > 0) {
			char msg[64];

			marklin_train_command_t command;
			command.command_type = MARKLIN_TRAIN_CMD_MANUAL_STOP;

			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Stopped train %d", train_num);
			} else {
				snprintf(msg, 64, "Failed to stop train %d: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: stop <train number>");
		}
	} else if (strncmp(command, "estop ", 6) == 0) {
		// Emergency stop command: estop <train>
		int train_num = 0;
		int pos = 6;

		train_num = parse_int(command, &pos);

		if (train_num > 0) {
			char msg[64];

			marklin_train_command_t command;
			command.command_type = MARKLIN_TRAIN_CMD_EMERGENCY_STOP;

			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Emergency stop for train %d", train_num);
			} else {
				snprintf(msg, 64, "Failed to emergency stop train %d: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: estop <train number>");
		}
		// } else if (strncmp(command, "offexp ", 7) == 0) {
		// 	// Offline experiment command: offexp <train> <type> <speeds...>
		// 	int pos = 7;
		// 	int train_num = parse_int(command, &pos);
		// 	char msg[128];
		// 	if (train_num > 0 && command[pos] == ' ') {
		// 		pos++; // Skip space
		// 		// Parse experiment type
		// 		char type_name[16] = { 0 };
		// 		int type_pos = 0;
		// 		while (command[pos] != '\0' && command[pos] != ' ' && type_pos < 15) {
		// 			type_name[type_pos] = command[pos];
		// 			type_pos++;
		// 			pos++;
		// 		}
		// 		type_name[type_pos] = '\0';

		// 		// Parse speeds
		// 		u8 speeds[16];
		// 		u8 speed_count = 0;
		// 		while (command[pos] != '\0' && speed_count < 16) {
		// 			if (command[pos] == ' ') {
		// 				pos++;
		// 				int speed = parse_int(command, &pos);
		// 				if (speed >= 0 && speed <= 14) {
		// 					speeds[speed_count++] = speed;
		// 				}
		// 			} else {
		// 				pos++;
		// 			}
		// 		}

		// 		if (speed_count > 0) {
		// 			marklin_train_command_t cmd;
		// 			cmd.command_type = MARKLIN_TRAIN_CMD_START_OFFLINE_EXPERIMENT;

		// 			if (strcmp(type_name, "vel") == 0) {
		// 				cmd.start_offline_experiment.experiment_type = OFFLINE_EXPERIMENT_VELOCITY_LOOP;
		// 				cmd.start_offline_experiment.speed_count = speed_count;
		// 				for (u8 i = 0; i < speed_count; i++) {
		// 					cmd.start_offline_experiment.speed_levels[i] = speeds[i];
		// 				}
		// 			} else if (strcmp(type_name, "accel") == 0) {
		// 				cmd.start_offline_experiment.experiment_type = OFFLINE_EXPERIMENT_ACCELERATION;
		// 				cmd.start_offline_experiment.speed_count = speed_count;
		// 				for (u8 i = 0; i < speed_count; i++) {
		// 					cmd.start_offline_experiment.speed_levels[i] = speeds[i];
		// 				}
		// 			} else if (strcmp(type_name, "stop") == 0) {
		// 				cmd.start_offline_experiment.experiment_type = OFFLINE_EXPERIMENT_STOP_DISTANCE;
		// 				cmd.start_offline_experiment.speed_count = speed_count;
		// 				for (u8 i = 0; i < speed_count; i++) {
		// 					cmd.start_offline_experiment.speed_levels[i] = speeds[i];
		// 				}
		// 			} else {
		// 				snprintf(msg, 128, "Invalid experiment type: %s (use vel, accel, or stop)",
		// 					 type_name);
		// 				tui_console_output(msg);
		// 				return;
		// 			}

		// 			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
		// 			if (result == MARKLIN_ERROR_OK) {
		// 				snprintf(msg, 128, "Started %s experiment for train %d", type_name, train_num);
		// 			} else {
		// 				snprintf(msg, 128, "Failed to start experiment for train %d: error %d",
		// 					 train_num, result);
		// 			}
		// 			tui_console_output(msg);
		// 		} else {
		// 			tui_console_output("Usage: offexp <train> <type> <speeds...>");
		// 		}
		// 	} else {
		// 		tui_console_output("Usage: offexp <train> <type> <speeds...>");
		// 	}
		// } else if (strncmp(command, "offstop ", 8) == 0) {
		// 	// Stop offline experiment command: offstop <train>
		// 	int train_num = 0;
		// 	int pos = 8;
		// 	char msg[64];
		// 	train_num = parse_int(command, &pos);
		// 	if (train_num > 0) {
		// 		marklin_train_command_t cmd;
		// 		cmd.command_type = MARKLIN_TRAIN_CMD_STOP_OFFLINE_EXPERIMENT;
		// 		marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
		// 		if (result == MARKLIN_ERROR_OK) {
		// 			snprintf(msg, 64, "Stopped offline experiment for train %d", train_num);
		// 		} else {
		// 			snprintf(msg, 64, "Failed to stop experiment for train %d: error %d", train_num,
		// 				 result);
		// 		}
		// 		tui_console_output(msg);
		// 	} else {
		// 		tui_console_output("Usage: offstop <train number>");
		// 	}
	} else if (strncmp(command, "model ", 6) == 0) {
		// Display kinematic model command: model <train>
		int train_num = 0;
		int pos = 6;
		char msg[64];
		train_num = parse_int(command, &pos);
		if (train_num > 0) {
			marklin_train_command_t cmd;
			cmd.command_type = MARKLIN_TRAIN_CMD_GET_KINEMATIC_MODEL;
			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
			if (result == MARKLIN_ERROR_OK) {
				// Model display will be handled by train task
				snprintf(msg, 64, "Displaying kinematic model for train %d", train_num);
			} else {
				snprintf(msg, 64, "Failed to get model for train %d: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: model <train number>");
		}
	} else if (strncmp(command, "debug ", 6) == 0) {
		// Debug command: debug <train>
		int train_num = 0;
		int pos = 6;
		char msg[64];
		train_num = parse_int(command, &pos);
		if (train_num > 0) {
			marklin_train_command_t cmd;
			cmd.command_type = MARKLIN_TRAIN_CMD_DEBUG_INFO;
			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Debug info printed for train %d", train_num);
			} else {
				snprintf(msg, 64, "Failed to get debug info for train %d: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: debug <train number>");
		}
	} else if (strncmp(command, "clear ", 6) == 0) {
		// Clear destination command: clear <train>
		int train_num = 0;
		int pos = 6;
		char msg[64];
		train_num = parse_int(command, &pos);
		if (train_num > 0) {
			marklin_train_command_t cmd;
			cmd.command_type = MARKLIN_TRAIN_CMD_CLEAR_DESTINATION;
			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &cmd);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Cleared destination for train %d", train_num);
			} else {
				snprintf(msg, 64, "Failed to clear destination for train %d: error %d", train_num, result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: clear <train number>");
		}
	} else if (strncmp(command, "random ", 7) == 0) {
		// Random destination command: random <train> <on/off>
		int pos = 7;
		int train_num = parse_int(command, &pos);
		char msg[64];

		if (train_num > 0 && command[pos] == ' ') {
			pos++; // Skip space

			// Parse on/off parameter
			bool enable_random = false;
			if (strncmp(&command[pos], "on", 2) == 0) {
				enable_random = true;
			} else if (strncmp(&command[pos], "off", 3) == 0) {
				enable_random = false;
			} else {
				tui_console_output("Usage: random <train number> <on|off>");
				return;
			}

			// Create the train command
			marklin_train_command_t command;
			command.command_type = MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE;
			command.set_random_destination_mode.enabled = enable_random;

			marklin_error_t result = Marklin_ControllerTrainCommand(train_num, &command);
			if (result == MARKLIN_ERROR_OK) {
				snprintf(msg, 64, "Random destination %s for train %d",
					 enable_random ? "enabled" : "disabled", train_num);
			} else {
				snprintf(msg, 64, "Failed to set random destination for train %d: error %d", train_num,
					 result);
			}
			tui_console_output(msg);
		} else {
			tui_console_output("Usage: random <train number> <on|off>");
		}
	} else if (strcmp(command, "blocks") == 0) {
		// Display block reservations command
		tui_display_block_reservations();
	} else if (strcmp(command, "q") == 0) {
		// Quit command
		tui_console_output("Shutting down and rebooting...");
		Delay(clock_server_tid, MS_TO_TICK(500));
		if (!tui_state.shell_mode) {
			tui_stop();
			console_puts("System rebooting...\n");
		}
		Reboot();
	} else if (strlen(command) > 0) {
		tui_console_output("Unknown command");
	}
}

static void tui_process_shell_command(void)
{
	tui_state.input_buffer[tui_state.input_pos] = '\0';

	if (tui_state.input_pos == 0) {
		return;
	}

	tui_process_command(tui_state.input_buffer);
}

// Record a recently triggered sensor
void tui_record_sensor_trigger(u8 bank, u8 sensor_num)
{
	// Get current time in ticks
	u64 current_tick = Time(clock_server_tid);

#define SENSOR_DEBOUNCE_WINDOW_TICKS MS_TO_TICK(1000) // 1 second debounce window

	for (int i = 0; i < MAX_RECENT_SENSORS; i++) {
		// Skip empty slots
		if (recent_sensors[i].last_trigger_tick == 0)
			continue;

		// If same sensor was triggered recently, ignore this trigger
		if (recent_sensors[i].bank == bank && recent_sensors[i].sensor_num == sensor_num &&
		    (current_tick - recent_sensors[i].last_trigger_tick) < SENSOR_DEBOUNCE_WINDOW_TICKS) {
			// klog_info("Ignoring duplicate trigger: bank %d, sensor %d (too soon)", bank, sensor_num);
			return;
		}
	}

	// Record the new sensor trigger
	recent_sensors[recent_sensor_index].bank = bank;
	recent_sensors[recent_sensor_index].sensor_num = sensor_num;
	recent_sensors[recent_sensor_index].last_trigger_tick = current_tick;

	// klog_info("Sensor triggered: bank %d, sensor %d, added to recent sensors @ %d", bank, sensor_num,
	//   recent_sensor_index);

	recent_sensor_index = (recent_sensor_index + 1) % MAX_RECENT_SENSORS;

	// Mark track panel for update and set dirty
	track_panel_needs_update = 1;
	tui_mark_panel_dirty(TUI_PANEL_TRACK);
}

// Function to mark track panel for update (can be called when train/switch data changes)
void tui_mark_track_panel_for_update(void)
{
	track_panel_needs_update = 1;
}

// Display current block reservation status
static void tui_display_block_reservations(void)
{
	// Initialize block status if needed
	tui_init_block_status();

	tui_console_output("Current Block Reservations:");
	tui_console_output("Block | Owner | Status     | Entry Sensor | Last Update");
	tui_console_output("------|-------|------------|--------------|------------");

	char line[128];
	u64 current_time = Time(clock_server_tid);

	for (int i = 0; i < MAX_BLOCKS; i++) {
		tui_block_status_t *block = &block_status[i];

		const char *status_str = "FREE";
		if (block->status == BLOCK_STATUS_RESERVED) {
			status_str = "RESERVED";
		} else if (block->status == BLOCK_STATUS_OCCUPIED) {
			status_str = "OCCUPIED";
		}

		// Calculate time since last update
		u64 time_since_update = 0;
		if (block->last_update_time > 0) {
			time_since_update = (current_time - block->last_update_time) / 100; // Convert to seconds
		}

		// Format the line
		if (block->owner_train_id == 0) {
			snprintf(line, sizeof(line), "%5d | %5s | %-10s | %-12s | %llu s ago", block->block_id, "-",
				 status_str, "-", time_since_update);
		} else {
			snprintf(line, sizeof(line), "%5d | %5d | %-10s | %-12s | %llu s ago", block->block_id,
				 block->owner_train_id, status_str,
				 block->entry_sensor_name[0] ? block->entry_sensor_name : "-", time_since_update);
		}

		tui_console_output(line);
	}

	tui_console_output(""); // Empty line for spacing
}

// Frame buffer implementation for batched terminal output
static void frame_buffer_init(void)
{
	frame_buffer_pos = 0;
	frame_buffer[0] = '\0';
}

static void frame_buffer_append(const char *data)
{
	if (!data)
		return;

	u32 data_len = strlen(data);
	if (frame_buffer_pos + data_len >= FRAME_BUFFER_SIZE - 1) {
		// Buffer full, flush it first
		frame_buffer_flush();
		frame_buffer_init();
	}

	strcpy(&frame_buffer[frame_buffer_pos], data);
	frame_buffer_pos += data_len;
}

static void frame_buffer_flush(void)
{
	if (frame_buffer_pos > 0) {
		console_printf("%s", frame_buffer);
		frame_buffer_init();
	}
}

static void frame_buffer_printf(const char *format, ...)
{
	char temp_buffer[512];
	va_list args;
	va_start(args, format);

	// Use __raw_vsnprintf which is available in this system
	int written = __raw_vsnprintf(temp_buffer, sizeof(temp_buffer), format, &args);
	va_end(args);

	if (written > 0) {
		frame_buffer_append(temp_buffer);
	}
}

// Update the track panel with switch and sensor information
void tui_update_track_panel(void)
{
	if (!tui_state.active) {
		return;
	}

	// Only update if the track panel actually needs updating
	if (!track_panel_needs_update) {
		return;
	}

	tui_panel_t *panel = &tui_state.panels[TUI_PANEL_TRACK];
	panel->buffer_pos = 0;

	// Calculate usable height: panel height - 2 (for borders)
	int usable_height = panel->height - 2;
	int lines_used = 0;

	// Get system snapshot to display train information
	marklin_system_snapshot_t system_snapshot = { 0 };
	marklin_error_t snapshot_result = Marklin_ControllerGetSystemSnapshot(&system_snapshot);

	char line[TUI_SCREEN_WIDTH * 4]; // Account for UTF-8 characters
	// int max_switch_count = (snapshot_result == MARKLIN_ERROR_OK) ? system_snapshot.active_switch_count : 0;
	int max_train_count = (snapshot_result == MARKLIN_ERROR_OK) ? system_snapshot.active_train_count : 0;

	// if (max_switch_count > 0) {
	// 	// Build the header row with switch numbers
	// 	tui_panel_add_message(TUI_PANEL_TRACK, "Switches:");
	// 	lines_used += 1;

	// 	// Ensure we show at least one switch column and limit to reasonable number
	// 	int display_switch_count = max_switch_count;
	// 	if (display_switch_count < 1)
	// 		display_switch_count = 1;
	// 	if (display_switch_count > 15)
	// 		display_switch_count = 15; // Reduced for safety

	// 	// Top border - use safe building
	// 	int pos = snprintf(line, sizeof(line), "┌─────");
	// 	for (int i = 1; i < display_switch_count && pos < (int)(sizeof(line) - 20); i++) {
	// 		pos += snprintf(&line[pos], sizeof(line) - pos, "┬─────");
	// 	}
	// 	if (pos < (int)(sizeof(line) - 10)) {
	// 		snprintf(&line[pos], sizeof(line) - pos, "┐");
	// 	}
	// 	tui_panel_add_message(TUI_PANEL_TRACK, line);
	// 	lines_used += 1;

	// 	// Switch numbers row
	// 	pos = 0;
	// 	for (int i = 0; i < display_switch_count && pos < (int)(sizeof(line) - 20); i++) {
	// 		if (i < max_switch_count) {
	// 			pos += snprintf(&line[pos], sizeof(line) - pos, "│ %3d ",
	// 					system_snapshot.switches[i].switch_id);
	// 		} else {
	// 			pos += snprintf(&line[pos], sizeof(line) - pos, "│  -  ");
	// 		}
	// 	}
	// 	if (pos < (int)(sizeof(line) - 10)) {
	// 		snprintf(&line[pos], sizeof(line) - pos, "│");
	// 	}
	// 	tui_panel_add_message(TUI_PANEL_TRACK, line);
	// 	lines_used += 1;

	// 	// Middle border
	// 	pos = snprintf(line, sizeof(line), "├─────");
	// 	for (int i = 1; i < display_switch_count && pos < (int)(sizeof(line) - 20); i++) {
	// 		pos += snprintf(&line[pos], sizeof(line) - pos, "┼─────");
	// 	}
	// 	if (pos < (int)(sizeof(line) - 10)) {
	// 		snprintf(&line[pos], sizeof(line) - pos, "┤");
	// 	}
	// 	tui_panel_add_message(TUI_PANEL_TRACK, line);
	// 	lines_used += 1;

	// 	// Direction row
	// 	pos = 0;
	// 	for (int i = 0; i < display_switch_count && pos < (int)(sizeof(line) - 20); i++) {
	// 		if (i < max_switch_count) {
	// 			const char *dir_str = (system_snapshot.switches[i].direction == DIR_STRAIGHT) ? "S" :
	// 													"C";
	// 			pos += snprintf(&line[pos], sizeof(line) - pos, "│  %s  ", dir_str);
	// 		} else {
	// 			pos += snprintf(&line[pos], sizeof(line) - pos, "│  -  ");
	// 		}
	// 	}
	// 	if (pos < (int)(sizeof(line) - 10)) {
	// 		snprintf(&line[pos], sizeof(line) - pos, "│");
	// 	}
	// 	tui_panel_add_message(TUI_PANEL_TRACK, line);
	// 	lines_used += 1;

	// 	// Bottom border
	// 	pos = snprintf(line, sizeof(line), "└─────");
	// 	for (int i = 1; i < display_switch_count && pos < (int)(sizeof(line) - 20); i++) {
	// 		pos += snprintf(&line[pos], sizeof(line) - pos, "┴─────");
	// 	}
	// 	if (pos < (int)(sizeof(line) - 10)) {
	// 		snprintf(&line[pos], sizeof(line) - pos, "┘");
	// 	}
	// 	tui_panel_add_message(TUI_PANEL_TRACK, line);
	// 	lines_used += 1;

	// 	// Add spacing
	// 	tui_panel_add_message(TUI_PANEL_TRACK, "");
	// 	lines_used += 1;
	// }

	// Display trains in a vertical table
	tui_panel_add_message(TUI_PANEL_TRACK,
			      "┌─────┬─────┬───┬───┬──────┬─────────┬─────────────┬─────────┬─────────┐");
	tui_panel_add_message(TUI_PANEL_TRACK,
			      "│ Trn │ Spd │ D │ L │ Mode │   Loc   │    Dest     │  Next   │  Status │");
	tui_panel_add_message(TUI_PANEL_TRACK,
			      "├─────┼─────┼───┼───┼──────┼─────────┼─────────────┼─────────┼─────────┤");
	lines_used += 4;

	// Limit train table rows to preserve space for sensor display
	// Reserve at least 6 lines for sensor section (separator + header + up to 3 sensors + footer)
	int max_train_rows = usable_height - lines_used - 6;
	if (max_train_rows < 1)
		max_train_rows = 1;
	if (max_train_count > max_train_rows) {
		max_train_count = max_train_rows;
	}

	if (max_train_count == 0) {
		// Show empty row
		tui_panel_add_message(TUI_PANEL_TRACK,
				      "│     │     │   │   │      │         │             │         │         │");
		tui_panel_add_message(TUI_PANEL_TRACK,
				      "│     │     │   │   │      │         │             │         │         │");
		lines_used += 2;
	} else {
		for (int i = 0; i < max_train_count; i++) {
			marklin_train_snapshot_t *train = &system_snapshot.trains[i];

			// Skip invalid trains (train_id = 0 means not spawned)
			if (train->train_id == 0) {
				continue;
			}
			char direction = (train->direction == TRAIN_DIRECTION_FORWARD) ? 'F' : 'R';
			char headlight = (train->headlight == MARKLIN_TRAIN_HEADLIGHT_ON) ? 'O' : 'X';

			// Get mode string
			const char *mode = (train->mode == TRAIN_MODE_MANUAL) ? "MAN" : "WPT";

			// Get location name with offset, use sensor name if available
			char location_with_offset[16];
			if (train->current_location && train->current_location->name) {
				if (train->location_offset_mm != 0) {
					snprintf(location_with_offset, 16, "%s+%d", train->current_location->name,
						 (int)train->location_offset_mm);
				} else {
					snprintf(location_with_offset, 16, "%s", train->current_location->name);
				}
			} else {
				snprintf(location_with_offset, 16, "Unknown");
			}

			char destination_with_offset[16];
			if (train->destination && train->destination->name) {
				if (train->destination_offset_mm != 0) {
					snprintf(destination_with_offset, 16, "%s+%d", train->destination->name,
						 (int)train->destination_offset_mm);
				} else {
					snprintf(destination_with_offset, 16, "%s", train->destination->name);
				}
			} else if (strlen(train->destination_name) > 0) {
				// Fallback to destination_name if no destination node
				snprintf(destination_with_offset, 16, "%s", train->destination_name);
			} else {
				snprintf(destination_with_offset, 16, "None");
			}

			// Get next sensors string
			char next_sensors[16];
			if (train->next_sensor_1 && train->next_sensor_2) {
				snprintf(next_sensors, 16, "%s,%s", train->next_sensor_1->name,
					 train->next_sensor_2->name);
			} else if (train->next_sensor_1) {
				snprintf(next_sensors, 16, "%s,-", train->next_sensor_1->name);
			} else {
				snprintf(next_sensors, 16, "-,-");
			}

			// Get status string
			const char *status;
			switch (train->status) {
			case TRAIN_STATUS_IDLE:
				status = "IDLE";
				break;
			case TRAIN_STATUS_REQUESTING_PATH:
				status = "REQ_PATH";
				break;
			case TRAIN_STATUS_MOVING:
				status = "MOVING";
				break;
			case TRAIN_STATUS_STOPPING:
				status = "STOPPING";
				break;
			default:
				status = "UNKNOWN";
				break;
			}

			snprintf(line, TUI_SCREEN_WIDTH, "│ %3d │  %2d │ %c │ %c │ %-4s │ %-7s │ %-11s │ %-7s │ %-7s │",
				 train->train_id, train->speed, direction, headlight, mode, location_with_offset,
				 destination_with_offset, next_sensors, status);
			tui_panel_add_message(TUI_PANEL_TRACK, line);
			lines_used += 1;
		}
		if (max_train_count == 1) {
			// If only one train, add an empty row for spacing
			tui_panel_add_message(
				TUI_PANEL_TRACK,
				"│     │     │   │   │      │         │             │         │         │");
			lines_used += 1;
		}
	}

	tui_panel_add_message(TUI_PANEL_TRACK,
			      "└─────┴─────┴───┴───┴──────┴─────────┴─────────────┴─────────┴─────────┘");
	lines_used += 1;

	// Add a separator
	tui_panel_add_message(TUI_PANEL_TRACK, "");

	// Build horizontal sensor triggers line
	char sensor_line[TUI_SCREEN_WIDTH];
	snprintf(sensor_line, TUI_SCREEN_WIDTH, "Recent Sensors: ");

	int count = 0;
	int max_sensor_count = 8; // Display up to 8 sensors horizontally

	for (int i = 0; i < MAX_RECENT_SENSORS && count < max_sensor_count; i++) {
		int idx = (recent_sensor_index - i - 1 + MAX_RECENT_SENSORS) % MAX_RECENT_SENSORS;

		if (recent_sensors[idx].last_trigger_tick > 0) {
			char sensor_str[16];
			char bank_char = 'A' + recent_sensors[idx].bank;

			snprintf(sensor_str, 16, "%c%d", bank_char, recent_sensors[idx].sensor_num);

			// Add sensor to horizontal line with separator
			if (count > 0) {
				strcat(sensor_line, ", ");
			}
			strcat(sensor_line, sensor_str);
			count++;
		}
	}

	if (count == 0) {
		strcat(sensor_line, "None");
	}

	tui_panel_add_message(TUI_PANEL_TRACK, sensor_line);
	lines_used += 2;

	// Add block reservation status table if there's space
	int remaining_lines = usable_height - lines_used;
	if (remaining_lines >= 8) { // Need at least 8 lines for a compact block table
		// Initialize block status if not done yet
		tui_init_block_status();

		// Add separator
		tui_panel_add_message(TUI_PANEL_TRACK, "");

		// Add block status table header
		tui_panel_add_message(TUI_PANEL_TRACK, "Block Status");

		// Display blocks in a 10x3 grid format for 30 blocks
		for (int row = 0; row < 3; row++) {
			char block_line[TUI_SCREEN_WIDTH];
			int pos = 0;

			for (int col = 0; col < 10; col++) {
				int block_id = row * 10 + col;
				if (block_id < MAX_BLOCKS) {
					tui_block_status_t *block = &block_status[block_id];

					// Determine display format based on block state
					if (block->status == BLOCK_STATUS_FREE) {
						// Free block: show just block ID and dash
						pos += snprintf(&block_line[pos], TUI_SCREEN_WIDTH - pos, "[%2d: -]  ",
								block_id);
					} else {
						// Reserved or occupied: show block ID and colored train ID
						const char *status_color;
						if (block->status == BLOCK_STATUS_RESERVED) {
							status_color = "\033[33m"; // Yellow for reserved
						} else { // BLOCK_STATUS_OCCUPIED
							status_color = "\033[31m"; // Red for occupied
						}

						pos += snprintf(&block_line[pos], TUI_SCREEN_WIDTH - pos,
								"[%2d:%s%2d\033[0m]  ", block_id, status_color,
								block->owner_train_id);
					}
				}
			}

			tui_panel_add_message(TUI_PANEL_TRACK, block_line);
		}
	}

	// Mark track panel as dirty and reset the update flag
	tui_mark_panel_dirty(TUI_PANEL_TRACK);
	track_panel_needs_update = 0;
}

// Update TUI with new data
void tui_update(void)
{
	// Check for key input even when TUI is not active
	int c = console_trygetc();
	if (c != IO_NO_DATA) {
		tui_process_input((char)c);
	}

	// Process message queue updates if subscribed
	if (sensor_subscription_active || block_subscription_active) {
		marklin_msgqueue_message_t message;
		marklin_error_t result = Marklin_MsgQueue_ReceiveNonBlock(&message);

		while (result == MARKLIN_ERROR_OK) {
			if (message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE) {
				tui_process_sensor_update(&message);
			} else if (message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION) {
				tui_process_block_reservation_update(&message);
			}

			// Try to get another message
			result = Marklin_MsgQueue_ReceiveNonBlock(&message);
		}
	}

	if (!tui_state.active) {
		return;
	}

	if (tui_state.shell_mode) {
		return;
	}

	u64 current_time_tick = Time(clock_server_tid);

	if (current_time_tick - tui_state.last_update_time_tick < MS_TO_TICK(TUI_MIN_UPDATE_INTERVAL_MS)) {
		return;
	}

	tui_state.last_update_time_tick = current_time_tick;

	tui_update_status();
	tui_update_track_panel();

	tui_draw();
	// Position cursor at input location with single IPC call (accounting for "> " prompt)
	console_printf("\033[%d;%dH", tui_state.panels[TUI_PANEL_INPUT].y + 2,
		       tui_state.panels[TUI_PANEL_INPUT].x + 2 + 2 + tui_state.input_pos); // +2 for "> "
}

void marklin_tui_server_task(void)
{
	tui_init();
	tui_start();

	while (1) {
		tui_update();
		Delay(clock_server_tid, 1);
	}
}

static int parse_int(const char *str, int *pos)
{
	int val = 0;
	int i = *pos;
	int negative = 0;

	while (str[i] == ' ' || str[i] == '\t') {
		i++;
	}

	// Check for negative sign
	if (str[i] == '-') {
		negative = 1;
		i++;
	}

	// Parse number
	if (str[i] >= '0' && str[i] <= '9') {
		while (str[i] >= '0' && str[i] <= '9') {
			val = val * 10 + (str[i] - '0');
			i++;
		}
		*pos = i;
		return negative ? -val : val;
	}

	return -1;
}

static char parse_char(const char *str, int *pos)
{
	int i = *pos;

	while (str[i] == ' ' || str[i] == '\t') {
		i++;
	}

	if (str[i] != '\0') {
		char c = str[i];
		*pos = i + 1;
		return c;
	}

	return '\0';
}

// Output text to the console area below the panels
static void tui_console_output(const char *msg)
{
	if (tui_state.shell_mode) {
		console_printf("%s\r\n", msg);
	} else {
		console_puts(SAVE_CURSOR);
		console_printf(CURSOR_POSITION "%s\r\n", console_output_current_y, 1, msg);

		console_output_current_y++;

		if (console_output_current_y > TUI_SCREEN_HEIGHT) {
			console_output_current_y = TUI_SCREEN_HEIGHT;
		}

		console_puts(RESTORE_CURSOR);
	}
}
