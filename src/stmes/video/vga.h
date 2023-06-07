#pragma once

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

// Ensure the alignment, so that ldmia/stmia instructions can be used for
// copying these. See <https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57648>.
// TODO: Verify if this really is the case.
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

#ifdef __cplusplus
}
#endif
