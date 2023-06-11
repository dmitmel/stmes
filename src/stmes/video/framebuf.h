#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIXEL_SCALE 2
#define FRAME_WIDTH (640 / PIXEL_SCALE)
#define FRAME_HEIGHT (480 / PIXEL_SCALE)
#define COLOR_BIT_DEPTH 12

struct PixelDmaBuffer {
  u32 data[FRAME_WIDTH];
  u32 non_zeroes[FRAME_WIDTH / 32];
  volatile u32* non_zeroes_bitband;
};

struct PixelDmaBuffer* swap_pixel_dma_buffers(void);

__STATIC_INLINE void pixel_dma_buf_set(struct PixelDmaBuffer* buf, u16 index, u32 value) {
  buf->data[index] = value;
  buf->non_zeroes_bitband[index] = 1;
}

void pixel_dma_buf_reset(struct PixelDmaBuffer* buf);

#ifdef __cplusplus
}
#endif
