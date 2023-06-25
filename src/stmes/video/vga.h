#pragma once

#include "stmes/gpio.h"
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

// Ensure the alignment, so that ldmia/stmia instructions can be used for
// copying these.
struct __ALIGNED(4) VgaScanline {
  u16 offset; // TODO
  u16 length;
  u16 repeats;
  u16 pixel_scale; // Starts at 0 for 1x scale (1 is 2x, 2 is 3x etc)
  u32* buffer;
};

struct VgaRegisters {
  volatile u16 scanline_request;
  volatile bool next_scanline_ready;
  volatile bool next_frame_request;
  volatile struct VgaScanline next_scanline;
};

extern struct VgaRegisters vga_registers;

__STATIC_FORCEINLINE bool vga_take_scanline_request(u16* line_nr) {
  u16 request = vga_registers.scanline_request;
  if (request != 0) {
    vga_registers.scanline_request = 0;
    *line_nr = request - 1;
    return true;
  }
  return false;
}

__STATIC_FORCEINLINE void vga_set_next_scanline(const struct VgaScanline* scanline) {
  vga_registers.next_scanline = *scanline;
  vga_registers.next_scanline_ready = true;
}

void vga_init(void);
void vga_apply_timings(const struct VgaTimings* ts);
void vga_start(void);
void vga_hsync_timer_isr(void);
void vga_vsync_timer_isr(void);
void vga_deinit(void);

// Since the GPIOB pins used for outputting the pixel color for VGA couldn't be
// chosen in a successive order (because PB4 and PB5 are taken up by SDIO data
// lines, and PB11 simply doesn't exist on the MCU chip package), we can't,
// e.g. just write the color's raw RGB value to the port's ODR register.
// Instead, this function establishes a mapping between the RGB colors and the
// pins corresponding to every single bit of the color, and returns a value
// suitable for writing to the BSRR register.
__STATIC_FORCEINLINE u32 rgb12_to_vga_pins(u32 color) {
  // Unfortunately I couldn't find a way to get the compiler to figure out the
  // bit airthmetic for me, so the masks and shifts in this function must be
  // recalculated in case the VGA pixel pins are changed.
  STATIC_ASSERT(VGA_PIXEL_ALL_PINS == 0x77CF);
  u32 pins = 0;
  pins |= (color & 0x00F) << 0; // color[ 3:0] -> pins[ 3:0 ]
  pins |= (color & 0x1F0) << 2; // color[ 8:4] -> pins[10:6 ]
  pins |= (color & 0xE00) << 3; // color[11:9] -> pins[14:12]
  pins |= (pins ^ 0x77CF) << 16;
  return pins;
}

#ifdef __cplusplus
}
#endif
