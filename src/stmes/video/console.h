#pragma once

#include "stmes/utils.h"
#include "stmes/video/console_font.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONSOLE_LINE_HEIGHT (CONSOLE_FONT_HEIGHT)
#define CONSOLE_COLUMNS (FRAME_WIDTH / CONSOLE_FONT_WIDTH)
#define CONSOLE_LINES (FRAME_HEIGHT / CONSOLE_LINE_HEIGHT)
#define CONSOLE_FRAME_WIDTH (CONSOLE_COLUMNS * CONSOLE_FONT_WIDTH)
#define CONSOLE_FRAME_HEIGHT (CONSOLE_LINES * CONSOLE_LINE_HEIGHT)
#define CONSOLE_TEXT_ATTRS_RESET 0x07

void console_set_color(u8 color);
void console_set_char(u8 line, u8 col, char c);
void console_clear_line(u8 line);
void console_clear_screen(void);
void console_new_line(void);
void console_feed_char(char c);
void console_init(void);
void console_render_scanline(u16 vga_line);
__NO_RETURN void console_main_loop(void);

#ifdef __cplusplus
}
#endif
