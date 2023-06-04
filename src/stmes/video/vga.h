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

extern TIM_HandleTypeDef vga_pixel_timer;
extern TIM_HandleTypeDef vga_hsync_timer;
extern TIM_HandleTypeDef vga_vsync_timer;
extern DMA_HandleTypeDef vga_pixel_dma;

void vga_init(void);
void vga_apply_timings(const struct VgaTimings* ts);
void vga_start(void);
void vga_deinit(void);

#ifdef __cplusplus
}
#endif
