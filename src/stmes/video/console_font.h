#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CONSOLE_FONT_WIDTH 6
#define CONSOLE_FONT_HEIGHT 12
#define CONSOLE_FONT_CHARACTERS 95
#define CONSOLE_FONT_BYTES_PER_ROW ((CONSOLE_FONT_WIDTH + 7) / 8)
#define CONSOLE_FONT_BYTES_PER_CHAR (CONSOLE_FONT_BYTES_PER_ROW * CONSOLE_FONT_HEIGHT)

extern const unsigned char CONSOLE_FONT_DATA[];

#ifdef __cplusplus
}
#endif
