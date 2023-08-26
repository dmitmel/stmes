#pragma once

#include "stmes/utils.h"
#include "stmes/video/vga_color.h"
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

#ifdef __cplusplus
}
#endif
