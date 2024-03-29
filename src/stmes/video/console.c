// TODO: Lock the state manipulation functions behind a mutex!

#include "stmes/video/console.h"
#include "stmes/utils.h"
#include "stmes/video/console_font.h"
#include "stmes/video/framebuf.h"
#include "stmes/video/vga.h"
#include "stmes/video/vga_color.h"
#include <printf.h>

// Multiplying a byte by this constant will duplicate it across the byte lanes.
#define SMEAR_8x4 0x01010101u

static struct ConsoleBuffer {
  // The 8-bit characters and attributes are stored in 32-bit bundles to allow
  // fetching 4 items with one aligned 32-bit load.
  char text[CONSOLE_LINES][(CONSOLE_COLUMNS + 3) / 4 * 4] __ALIGNED(4);
  // Same goes for text attributes. The attributes themselves are encoded as
  // follows: the 4 low bits are for the foregound color, the high 4 are for
  // the background.
  u8 text_attrs[CONSOLE_LINES][(CONSOLE_COLUMNS + 3) / 4 * 4] __ALIGNED(4);
  // Also, the text lines array actually forms a ring buffer to enable quick
  // scrolling of the console buffer.
  u8 top_line;
  u8 cursor_col, cursor_line, current_text_attrs;
} console_buffer = { 0 };

// These colors are converted into pixel pin values below.
static VgaPixel console_palette[16] = {
  0x000, 0xF00, 0x0F0, 0xFF0, 0x00F, 0xF0F, 0x0FF, 0xFFF,
  0x444, 0x800, 0x080, 0x880, 0x008, 0x808, 0x088, 0x888,
};

u8 console_get_current_color(void) {
  return console_buffer.current_text_attrs;
}

void console_set_color(u8 color) {
  console_buffer.current_text_attrs = color;
}

void console_set_char(u8 line, u8 col, char c) {
  struct ConsoleBuffer* self = &console_buffer;
  self->text[line][col] = c;
  self->text_attrs[line][col] = self->current_text_attrs;
}

void console_set_cursor_line(u8 line) {
  struct ConsoleBuffer* self = &console_buffer;
  self->cursor_line = (self->top_line + line) % CONSOLE_LINES;
}

void console_clear_line(u8 line) {
  struct ConsoleBuffer* self = &console_buffer;
  u8 attr = self->current_text_attrs;
  // Note how we can *assign* four characters at a time as well.
  fast_memset_u32((u32*)self->text[line], ' ' * SMEAR_8x4, SIZEOF(self->text[0]) / 4);
  fast_memset_u32((u32*)self->text_attrs[line], attr * SMEAR_8x4, SIZEOF(self->text_attrs[0]) / 4);
}

void console_clear_cursor_line(void) {
  struct ConsoleBuffer* self = &console_buffer;
  console_clear_line(self->cursor_line);
}

void console_clear_screen(void) {
  struct ConsoleBuffer* self = &console_buffer;
  self->cursor_col = self->cursor_line = self->top_line = 0;
  self->current_text_attrs = CONSOLE_TEXT_ATTRS_RESET;
  for (usize line = 0; line < CONSOLE_LINES; line++) {
    console_clear_line(line);
  }
}

void console_new_line(void) {
  struct ConsoleBuffer* self = &console_buffer;
  self->cursor_line = (self->cursor_line + 1) % CONSOLE_LINES;
  if (self->cursor_line == self->top_line) {
    self->top_line = (self->top_line + 1) % CONSOLE_LINES;
    console_clear_line(self->cursor_line);
  }
  self->cursor_col = 0;
}

void console_putchar(char c) {
  struct ConsoleBuffer* self = &console_buffer;
  if (c == '\n') {
    console_new_line();
  } else if (c == '\r') {
    self->cursor_col = 0;
  } else {
    console_set_char(self->cursor_line, self->cursor_col, c);
    self->cursor_col += 1;
    if (self->cursor_col >= CONSOLE_COLUMNS) {
      console_new_line();
    }
  }
}

void console_print(const char* str) {
  while (*str != 0) {
    console_putchar(*str++);
  }
}

void console_setup_frame_config(void) {
  struct VgaFrameConfig frame = {
    .line_length = FRAME_WIDTH,
    .lines_count = FRAME_HEIGHT * PIXEL_SCALE,
    .pixel_scale = PIXEL_SCALE - 1,
    .line_repeats = PIXEL_SCALE - 1,
  };
  vga_set_frame_config(&frame);
}

// GCC_ATTRIBUTE(optimize("-Os"))
bool console_render_scanline(u16 vga_line) {
  usize y = vga_line / PIXEL_SCALE - (FRAME_HEIGHT - CONSOLE_FRAME_HEIGHT) / 2;
  if (y >= CONSOLE_FRAME_HEIGHT) return false;

  struct ConsoleBuffer* self = &console_buffer;
  struct PixelDmaBuffer* backbuf = swap_pixel_dma_buffers();
  vga_set_next_scanline(backbuf->data);

  usize line_nr = y / CONSOLE_LINE_HEIGHT, char_y = y % CONSOLE_LINE_HEIGHT;
  usize x = (FRAME_WIDTH - CONSOLE_FRAME_WIDTH) / 2;
  if (char_y >= CONSOLE_FONT_HEIGHT) {
    vga_fast_memset(backbuf->data, 0, FRAME_WIDTH);
    return false;
  }

  vga_fast_memset(backbuf->data, 0, x);
  usize last_x = x + CONSOLE_FRAME_WIDTH;
  vga_fast_memset(backbuf->data + last_x, 0, FRAME_WIDTH - last_x);

  // Apply the vertical scroll.
  line_nr = (self->top_line + line_nr) % CONSOLE_LINES;

  // NOTE: The timing is CRITICAL in the code below!!!

  VgaPixel* pixel_ptr = backbuf->data + x;
  const u32* text = (const u32*)self->text[line_nr];
  const u32* attrs = (const u32*)self->text_attrs[line_nr];
  usize cursor_col = self->cursor_line == line_nr ? self->cursor_col : UINTPTR_MAX;
  // The font is stored in a row-major order (i.e. first comes the block for
  // y=0 row of every character, then the block for the row at y=1 and so on
  // and so on) to improve cache locality because we are going to be accessing
  // only the same single vertical slice of each character.
  const u8* font_row_data = &CONSOLE_FONT_DATA[char_y * CONSOLE_FONT_CHARACTERS];
  for (usize col_nr = 0; col_nr < CONSOLE_COLUMNS; col_nr += 4) {
    usize actually_fetched_chars = MIN(4, CONSOLE_COLUMNS - col_nr);
    // Note that here we fetch 4 characters and attributes at once.
    u32 chars = *text++;
    u32 char_attrs = *attrs++;

    // Check if the cursor is inside this group of 4 characters.
    if (cursor_col / 4 == col_nr / 4) {
      // If so, locate the one under the cursor and replace its attributes.
      u32 shift = (cursor_col % 4) * 8;
      char_attrs &= ~(MASK(8) << shift);
      char_attrs |= (CONSOLE_CURSOR_TEXT_ATTRS << shift);
    }

    // Now comes a bit of SIMD trickery. First, we subtract (in parallel) 0x20
    // from every character (since the font doesn't include the control
    // characters and starts at 32nd one). Note that if the character was one
    // of those 32, the result will overflow.
    chars = __USUB8(chars, SMEAR_8x4 * 0x20);
    // Next we compare every obtained character index with the total number of
    // characters in the font. This is done by exploiting the fact that the
    // SIMD operations for parallel addition/subtraction set flags on the
    // special APSR.GE register depending on whether the result has overflowed.
    __USUB8(SMEAR_8x4 * CONSOLE_FONT_CHARACTERS, chars);
    // And, finally, we select the characters which passed the condition above
    // (char_idx < CONSOLE_FONT_CHARACTERS), replacing those that didn't. The
    // logic here is the usub8 instruction above will set the APSR.GE flags
    // corresponding to each byte of the result that is >= 0, and consequently,
    // the sel instruction chooses (in parallel) between the bytes of two
    // values, picking a byte from the first value if its corresponding flag is
    // set, and otherwise selecting one from the second value.
    chars = __SEL(chars, SMEAR_8x4 * ('?' - 0x20));
    // For more info on the available SIMD operations see
    // <https://www.keil.com/pack/doc/CMSIS/Core/html/group__intrinsic__SIMD__gr.html>

    // It's a shame that there is no "parallel load" instruction, so now we
    // have to unpack the u32 with the character values to load the font bytes.
    usize char_idx1 = (chars >> 0) & 0xFF, char_idx2 = (chars >> 8) & 0xFF;
    usize char_idx3 = (chars >> 16) & 0xFF, char_idx4 = (chars >> 24) & 0xFF;
    // Compute the indexes before performing the loads so that the two lines
    // below compile to consecutive `ldrb` instructions, which may be pipelined.
    u8 font_byte1 = font_row_data[char_idx1], font_byte2 = font_row_data[char_idx2];
    u8 font_byte3 = font_row_data[char_idx3], font_byte4 = font_row_data[char_idx4];

    // Afterwards we (again) pack all font bytes in a single contiguous u32 to
    // be able to process all pixel data in a single loop below.
    u32 font_bytes = font_byte1 | (font_byte2 << 8) | (font_byte3 << 16) | (font_byte4 << 24);

    // Aaaaaand finally we can output some pixels!
    for (VgaPixel* end = pixel_ptr + CONSOLE_FONT_WIDTH * actually_fetched_chars; pixel_ptr != end;
         font_bytes >>= 8, char_attrs >>= 8) {
      // Same story with the indexes and loads as above.
      usize fg_idx = char_attrs & MASK(4), bg_idx = (char_attrs >> 4) & MASK(4);
      // The RAM cache does a pretty good job at keeping these around for
      // strides of characters with the same colors.
      VgaPixel fg = console_palette[fg_idx], bg = console_palette[bg_idx];
      if ((font_bytes & 0xFF) != 0) {
        usize i = 0;
        // I'm using the poor man's loop unroller here (the macro simply
        // copy-pastes its argument N times), however, the compiler seems to do
        // a really good job at translating this, expanding the loop variables
        // into constants and removing the unnecessary iterations.
        UNROLL_8({
          // Also, apparently my non-zero color value optimization is not worth
          // it here - just dumb copying and assigning every pixel is twice as
          // fast than anything I could come up with for detecting and skipping
          // the unchanged pixel values.
          if (i++ < CONSOLE_FONT_WIDTH) {
            // This should more or less compile to 3 instructions per pixel:
            //
            // sub fg_bg, fg, bg
            // ...
            // ubfx bit, font_bytes, #<i>, #1
            // mla color, bit, fg_bg, bg
            // str color, [pixel_ptr], #4
            // ...
            u32 bit = (font_bytes >> (8 - i)) & 1;
            // This expression selects between fg and bg depending on whether
            // the bit is 1 or 0 respectively.
            *pixel_ptr++ = (fg - bg) * bit + bg;
          }
        });
      } else {
        usize i = 0;
        // A fast path for blank characters, speeds up the rendering in such
        // cases by ~33%.
        UNROLL_8({
          if (i++ < CONSOLE_FONT_WIDTH) *pixel_ptr++ = bg;
        });
      }
    }
  }

  return true;
}

void console_init(void) {
  for (usize i = 0; i < SIZEOF(console_palette); i++) {
    console_palette[i] = rgb12_to_vga_pins(console_palette[i]);
  }
  console_clear_screen();
}
