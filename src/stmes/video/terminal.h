#pragma once

#include "stmes/utils.h"
#include "stmes/video/framebuf.h"
#include "stmes/video/terminal_font.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TERMINAL_LINE_HEIGHT (TERMINAL_FONT_HEIGHT)
#define TERMINAL_COLUMNS (FRAME_WIDTH / TERMINAL_FONT_WIDTH)
#define TERMINAL_LINES (FRAME_HEIGHT / TERMINAL_LINE_HEIGHT)
#define TERMINAL_FRAME_WIDTH (TERMINAL_COLUMNS * TERMINAL_FONT_WIDTH)
#define TERMINAL_FRAME_HEIGHT (TERMINAL_LINES * TERMINAL_LINE_HEIGHT)
#define TERMINAL_TEXT_ATTRS_RESET 0x07
#define TERMINAL_CURSOR_TEXT_ATTRS 0x70

u8 terminal_get_current_color(void);
void terminal_set_color(u8 color);
void terminal_set_char(u8 line, u8 col, char c);
void terminal_set_cursor_line(u8 line);
void terminal_clear_line(u8 line);
void terminal_clear_cursor_line(void);
void terminal_clear_screen(void);
void terminal_new_line(void);
void terminal_putchar(char c);
void terminal_print(const char* str);
void terminal_init(void);
void terminal_setup_frame_config(void);
bool terminal_render_scanline(u16 vga_line);

extern struct Task terminal_drawing_task;
void start_terminal_drawing_task(void);

#ifdef __cplusplus
}
#endif
