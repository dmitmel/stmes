#pragma once

#include "stmes/gpio.h"
#include "stmes/kernel/task.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

struct VgaTimings {
  u32 pixel_clock_freq;
  u16 horz_front_porch, vert_front_porch;
  u16 active_width, active_height;
  i16 hsync_pulse, vsync_pulse; // >0 - positive pulse polarity, <0 - negative pulse polarity
  u16 horz_back_porch, vert_back_porch;
};

extern const struct VgaTimings VGA_TIMINGS_640x480_57hz;
extern const struct VgaTimings VGA_TIMINGS_640x480_60hz;
extern const struct VgaTimings VGA_TIMINGS_800x600_60hz;
extern const struct VgaTimings VGA_TIMINGS_1024x768_60hz;

typedef u32 VgaPixel;
#define vga_fast_memset fast_memset_u32
#define vga_fast_memcpy fast_memcpy_u32

// Ensure the alignment, so that ldmia/stmia instructions can be used for
// copying instances of these.
struct __ALIGNED(4) VgaFrameConfig {
  u16 pixel_scale; // Starts at 0 for 1x scale (1 is 2x, 2 is 3x etc)
  u16 line_repeats;
  u16 offset_left;
  u16 offset_top;
  u16 line_length;
  u16 lines_count;
};

extern volatile struct VgaControlBlock {
  bool entering_frame;
  bool entering_vblank;
  bool next_scanline_requested;
  bool frame_config_ready;
  u16 next_scanline_nr;
  const VgaPixel* next_scanline;
  struct VgaFrameConfig frame_config;
} vga_control;

extern struct Notification vga_notification;

__STATIC_FORCEINLINE bool vga_take_scanline_request(u16* line_nr) {
  if (vga_control.next_scanline_requested) {
    vga_control.next_scanline_requested = false;
    *line_nr = vga_control.next_scanline_nr;
    return true;
  }
  return false;
}

__STATIC_FORCEINLINE void vga_set_next_scanline(const VgaPixel* scanline) {
  vga_control.next_scanline = scanline;
}

__STATIC_FORCEINLINE void vga_set_frame_config(const struct VgaFrameConfig* cfg) {
  volatile struct VgaFrameConfig* dst = &vga_control.frame_config;
  // Copying these field-by-field generates better assembly than assigning an
  // entire struct. Plus, apparently, in C++ you can't assign volatile structs.
  dst->pixel_scale = cfg->pixel_scale;
  dst->line_repeats = cfg->line_repeats;
  dst->offset_left = cfg->offset_left;
  dst->offset_top = cfg->offset_top;
  dst->line_length = cfg->line_length;
  dst->lines_count = cfg->lines_count;
  vga_control.frame_config_ready = true;
}

void vga_init(void);
void vga_apply_timings(const struct VgaTimings* ts);
void vga_start(void);
void vga_deinit(void);

// Since the GPIOB pins used for outputting the pixel color for VGA couldn't be
// chosen in a successive order (because PB4 and PB5 are taken up by SDIO data
// lines, and PB11 simply doesn't exist on the MCU chip package), we can't,
// e.g. just write the color's raw RGB value to the port's ODR register.
// Instead, this function establishes a mapping between the RGB colors and the
// pins corresponding to every single bit of the color, and returns a value
// suitable for writing to the BSRR register.
__STATIC_FORCEINLINE VgaPixel rgb12_to_vga_pins(VgaPixel color) {
  // Unfortunately I couldn't find a way to get the compiler to figure out the
  // bit airthmetic for me, so the masks and shifts in this function must be
  // recalculated in case the VGA pixel pins are changed.
  STATIC_ASSERT(VGA_PIXEL_ALL_PINS == 0x77CF);
  VgaPixel pins = 0;
  pins |= (color & 0x00F) << 0; // color[ 3:0] -> pins[ 3:0 ]
  pins |= (color & 0x1F0) << 2; // color[ 8:4] -> pins[10:6 ]
  pins |= (color & 0xE00) << 3; // color[11:9] -> pins[14:12]
  pins |= (pins ^ 0x77CF) << 16;
  return pins;
}

__STATIC_FORCEINLINE u32 rgb24_to_rgb12(u32 rgb) {
  u8 r8 = rgb >> 16, g8 = rgb >> 8, b8 = rgb >> 0;
  u8 r4 = r8 >> 4, g4 = g8 >> 4, b4 = b8 >> 4;
  return (r4 << 8) | (g4 << 4) | (b4 << 0);
}

#ifdef __cplusplus
}
#endif
