#include "stmes/video/framebuf.h"

static struct PixelDmaBuffer pixel_dma_buf_1 = { 0 }, pixel_dma_buf_2 = { 0 };
static bool pixel_dma_buffers_swapped = false;

struct PixelDmaBuffer* swap_pixel_dma_buffers(void) {
  pixel_dma_buffers_swapped = !pixel_dma_buffers_swapped;
  struct PixelDmaBuffer* buf = pixel_dma_buffers_swapped ? &pixel_dma_buf_1 : &pixel_dma_buf_2;
  buf->non_zeroes_bitband = SRAM1_BITBAND_ADDR(buf->non_zeroes, 0);
  return buf;
}

void pixel_dma_buf_reset(struct PixelDmaBuffer* buf) {
  VgaPixel* pixel_ptr = buf->data;
  for (usize i = 0; i < SIZEOF(buf->non_zeroes); i++) {
    u32 non_zeroes = buf->non_zeroes[i];
    buf->non_zeroes[i] = 0;
    if (non_zeroes & 1) {
      non_zeroes &= ~BIT(0);
      *pixel_ptr = 0;
    }
    pixel_ptr += 30;
    while (non_zeroes != 0) {
      u32 offset = __CLZ(non_zeroes);
      // sizeof values are compile-time constants, this will be collapsed into
      // a single branch even with the optimizations turned off.
      switch (sizeof(*pixel_ptr)) {
        case sizeof(u32): *(u64*)(pixel_ptr - offset) = 0; break;
        case sizeof(u16): *(u32*)(pixel_ptr - offset) = 0; break;
        case sizeof(u8): *(u16*)(pixel_ptr - offset) = 0; break;
      }
      non_zeroes &= ~(BIT(31) | BIT(30)) >> offset;
      // non_zeroes ^= BIT(31) >> offset;
    }
    pixel_ptr += 2;
  }
}
