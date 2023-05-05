#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void SystemClock_Config(void);

void Error_Handler(void);

enum __packed SyncPolarity {
  SYNC_PULSE_POLARITY_POSITIVE,
  SYNC_PULSE_POLARITY_NEGATIVE,
};

struct VgaTiming {
  u32 pixel_freq_hz;
  u16 horz_front_porch, vert_front_porch;
  u16 visible_width, visible_height;
  u16 hsync_pulse, vsync_pulse;
  u16 horz_back_porch, vert_back_porch;
  enum SyncPolarity hsync_polarity, vsync_polarity;
};

extern volatile bool inside_frame_vertical, inside_frame_horizontal;
extern volatile u32 frame_counter;

#ifdef __cplusplus
}
#endif
