#ifndef MARKLIN_TUI_H
#define MARKLIN_TUI_H

#include "types.h"

// Panel identifiers
#define TUI_PANEL_STATUS 0
#define TUI_PANEL_TRACK 1
#define TUI_PANEL_INPUT 2
#define TUI_PANEL_COUNT 3

#define TUI_SCREEN_WIDTH 130
#define TUI_SCREEN_HEIGHT 50

// Panel styles
#define TUI_STYLE_BORDER 1
#define TUI_STYLE_NO_BORDER 0

// Colors (ANSI escape codes)
#define TUI_COLOR_BLACK "\033[30m"
#define TUI_COLOR_RED "\033[31m"
#define TUI_COLOR_GREEN "\033[32m"
#define TUI_COLOR_YELLOW "\033[33m"
#define TUI_COLOR_BLUE "\033[34m"
#define TUI_COLOR_MAGENTA "\033[35m"
#define TUI_COLOR_CYAN "\033[36m"
#define TUI_COLOR_WHITE "\033[37m"
#define TUI_COLOR_RESET "\033[0m"
#define TUI_BG_BLACK "\033[40m"
#define TUI_BG_RED "\033[41m"
#define TUI_BG_GREEN "\033[42m"
#define TUI_BG_YELLOW "\033[43m"
#define TUI_BG_BLUE "\033[44m"
#define TUI_BG_MAGENTA "\033[45m"
#define TUI_BG_CYAN "\033[46m"
#define TUI_BG_WHITE "\033[47m"

// Special keys
#define TUI_KEY_ENTER '\r'
#define TUI_KEY_ESCAPE 27
#define TUI_KEY_BACKSPACE 127

#define TUI_F1_STATE_NORMAL 0
#define TUI_F1_STATE_ESC 1
#define TUI_F1_STATE_BRACKET 2
#define TUI_F1_STATE_O 3

// Panel structure
typedef struct {
	u32 x; // X position (top-left corner)
	u32 y; // Y position (top-left corner)
	u32 width; // Width in characters
	u32 height; // Height in characters
	u8 style; // Panel style
	char title[16]; // Panel title
	char *buffer; // Panel content buffer
	u64 buffer_size; // Size of the buffer
	u64 buffer_pos; // Current position in buffer
	u8 dirty; // Flag indicating the panel needs to be redrawn
	u8 border_drawn; // Flag indicating if border has been drawn
	char *last_buffer; // Copy of last drawn buffer for dirty region detection
	u64 last_buffer_pos; // Last drawn buffer position
} tui_panel_t;

// TUI state
typedef struct {
	int tid;
	u8 active;
	u8 shell_mode; // 0 = TUI mode, 1 = shell mode
	u8 f1_state; // State for F1 key sequence detection
	tui_panel_t panels[TUI_PANEL_COUNT];
	char input_buffer[128];
	u8 input_pos;
	u64 klog_last_index;
	u64 last_update_time_tick;
} tui_state_t;

// Initialize TUI interface
void tui_init(void);

// Update TUI with new data
void tui_update(void);

// Process user input
void tui_process_input(char c);

// Draw the entire screen
void tui_draw(void);

// Draw a specific panel
void tui_draw_panel(u8 panel_id);

// Mark a panel as dirty (needs redraw)
void tui_mark_panel_dirty(u8 panel_id);

// Clear the screen
void tui_clear_screen(void);

// Set cursor position
void tui_set_cursor(u8 x, u8 y);

// Add a message to a panel
void tui_panel_add_message(u8 panel_id, const char *msg);

// Add text at a specific position in a panel
void tui_panel_add_text(u8 panel_id, u8 x, u8 y, const char *msg);

// Update status information
void tui_update_status(void);

// Start TUI interface
void tui_start(void);

// Stop TUI interface
void tui_stop(void);

// Toggle between TUI and shell modes
void tui_toggle_mode(void);

// Force a full redraw of all panels
void tui_force_redraw(void);

// Record a triggered sensor for the track panel
void tui_record_sensor_trigger(u8 bank, u8 sensor_num);

// Mark track panel for update (call when train/switch data changes)
void tui_mark_track_panel_for_update(void);

#define MARKLIN_TUI_SERVER_TASK_PRIORITY 10
void marklin_tui_server_task(void);

#endif /* MARKLIN_TUI_H */
