#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

struct DmaConfig {
#if defined(DMA_SxCR_CHSEL_3)
  u32 channel : 4;
#else
  u32 channel : 3;
  u32 : 1; // padding
#endif
  enum __packed DmaPriority {
    DMA_LOW_PRIORITY = 0,
    DMA_MEDIUM_PRIORITY = 1,
    DMA_HIGH_PRIORITY = 2,
    DMA_VERY_HIGH_PRIORITY = 3,
  } priority : 2;
  enum __packed DmaDirection {
    DMA_PERIPH_TO_MEMORY_DIR = 0,
    DMA_MEMORY_TO_PERIPH_DIR = 1,
    DMA_MEMORY_TO_MEMORY_DIR = 2,
  } direction : 2;
  enum __packed DmaMode {
    DMA_NORMAL_MODE = 0,
    DMA_CIRCULAR_MODE = 1,
    DMA_PERIPHERAL_FLOW_CONTROL = 2,
  } mode : 2;
  enum __packed DmaDataSize {
    DMA_BYTE_DATA = 0,
    DMA_HALFWORD_DATA = 1,
    DMA_WORD_DATA = 2
  } periph_data_size : 2,
    memory_data_size : 2;
  bool periph_addr_increment : 1, memory_addr_increment : 1;
  bool double_buffer_mode : 1;
  bool enable_fifo : 1;
  enum __packed DmaFifoThreshold {
    DMA_1QUARTER_FULL_FIFO_THRESHOLD = 0,
    DMA_HALF_FULL_FIFO_THRESHOLD = 1,
    DMA_3QUARTERS_FULL_FIFO_THRESHOLD = 2,
    DMA_FULL_FIFO_THRESHOLD = 3,
  } fifo_threshold : 2;
  enum __packed DmaBurstSize {
    DMA_BURST_SINGLE = 0,
    DMA_BURST_INCR4 = 1,
    DMA_BURST_INCR8 = 2,
    DMA_BURST_INCR16 = 3,
  } periph_burst : 2,
    memory_burst : 2;
};

#define DMA_FLAG_FEIF BIT(0)
#define DMA_FLAG_DMEIF BIT(2)
#define DMA_FLAG_TEIF BIT(3)
#define DMA_FLAG_HTIF BIT(4)
#define DMA_FLAG_TCIF BIT(5)
#define DMA_ALL_INTERRUPT_FLAGS \
  (DMA_FLAG_FEIF | DMA_FLAG_DMEIF | DMA_FLAG_TEIF | DMA_FLAG_HTIF | DMA_FLAG_TCIF)

__STATIC_FORCEINLINE DMA_TypeDef* dma_get_base_registers(DMA_Stream_TypeDef* dma) {
  // RM0383 section 2.3 "Memory map"
  // <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c#L1201>
  return (DMA_TypeDef*)((usize)dma & ~0x3FF);
}

__STATIC_FORCEINLINE usize dma_get_stream_index(DMA_Stream_TypeDef* dma) {
  // RM0383 section 9.5.11 "DMA register map"
  // <https://github.com/STMicroelectronics/STM32CubeF4/blob/v1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c#L1187>
  return (((usize)dma & 0xFF) - sizeof(DMA_TypeDef)) / sizeof(DMA_Stream_TypeDef);
}

__STATIC_FORCEINLINE u32 dma_get_interrupt_flags(DMA_Stream_TypeDef* dma) {
  DMA_TypeDef* base = dma_get_base_registers(dma);
  usize idx = dma_get_stream_index(dma);
  static const u8 flags_offset[] = { 0, 6, 16, 22 };
  // u32 isr = (idx & BIT(2)) ? base->HISR : base->LISR;
  // if (idx & BIT(1)) isr >>= 16;
  // if (idx & BIT(0)) isr >>= 6;
  u32 isr = (&base->LISR)[(idx & BIT(2)) ? 1 : 0];
  isr >>= flags_offset[idx & MASK(2)];
  return isr & MASK(6);
}

__STATIC_FORCEINLINE void dma_clear_interrupt_flags(DMA_Stream_TypeDef* dma, u32 mask) {
  DMA_TypeDef* base = dma_get_base_registers(dma);
  usize idx = dma_get_stream_index(dma);
  static const u8 flags_offset[] = { 0, 6, 16, 22 };
  mask &= MASK(6);
  // if (idx & BIT(0)) mask <<= 6;
  // if (idx & BIT(1)) mask <<= 16;
  // (idx & BIT(2)) ? (base->HIFCR = mask) : (base->LIFCR = mask);
  mask <<= flags_offset[idx & MASK(2)];
  (&base->LIFCR)[(idx & BIT(2)) ? 1 : 0] = mask;
}

__STATIC_FORCEINLINE bool dma_is_stream_enabled(const DMA_Stream_TypeDef* dma) {
  return READ_BIT(dma->CR, DMA_SxCR_EN) != 0;
}

__STATIC_FORCEINLINE void dma_enable_stream(DMA_Stream_TypeDef* dma) {
  SET_BIT(dma->CR, DMA_SxCR_EN);
}

__STATIC_FORCEINLINE void dma_disable_stream(DMA_Stream_TypeDef* dma) {
  CLEAR_BIT(dma->CR, DMA_SxCR_EN);
}

__STATIC_FORCEINLINE void
dma_configure_stream(DMA_Stream_TypeDef* dma, const struct DmaConfig* conf) {
  u32 cr = READ_REG(dma->CR), fcr = READ_REG(dma->FCR);
  MODIFY_REG(cr, DMA_SxCR_CHSEL, conf->channel << DMA_SxCR_CHSEL_Pos);
  MODIFY_REG(cr, DMA_SxCR_PL, conf->priority << DMA_SxCR_PL_Pos);
  MODIFY_REG(cr, DMA_SxCR_DIR, conf->direction << DMA_SxCR_DIR_Pos);
  MODIFY_REG(cr, DMA_SxCR_CIRC, conf->mode == DMA_CIRCULAR_MODE ? DMA_SxCR_CIRC : 0);
  MODIFY_REG(cr, DMA_SxCR_PFCTRL, conf->mode == DMA_PERIPHERAL_FLOW_CONTROL ? DMA_SxCR_PFCTRL : 0);
  MODIFY_REG(cr, DMA_SxCR_PSIZE, conf->periph_data_size << DMA_SxCR_PSIZE_Pos);
  MODIFY_REG(cr, DMA_SxCR_MSIZE, conf->memory_data_size << DMA_SxCR_MSIZE_Pos);
  MODIFY_REG(cr, DMA_SxCR_PINC, conf->periph_addr_increment ? DMA_SxCR_PINC : 0);
  MODIFY_REG(cr, DMA_SxCR_MINC, conf->memory_addr_increment ? DMA_SxCR_MINC : 0);
  MODIFY_REG(cr, DMA_SxCR_DBM, conf->double_buffer_mode ? DMA_SxCR_DBM : 0);
  MODIFY_REG(fcr, DMA_SxFCR_DMDIS, conf->enable_fifo ? DMA_SxFCR_DMDIS : 0);
  MODIFY_REG(fcr, DMA_SxFCR_FTH, conf->fifo_threshold << DMA_SxFCR_FTH_Pos);
  MODIFY_REG(cr, DMA_SxCR_PBURST, conf->periph_burst << DMA_SxCR_PBURST_Pos);
  MODIFY_REG(cr, DMA_SxCR_MBURST, conf->memory_burst << DMA_SxCR_MBURST_Pos);
  CLEAR_BIT(cr, DMA_SxCR_CT); // Reset the current target in double-buffered mode
  WRITE_REG(dma->CR, cr), WRITE_REG(dma->FCR, fcr);
}

__STATIC_FORCEINLINE void dma_configure_transfer(
  DMA_Stream_TypeDef* dma, usize peripheral_addr, usize memory_addr, u16 length
) {
  WRITE_REG(dma->NDTR, length);
  WRITE_REG(dma->PAR, (usize)peripheral_addr);
  WRITE_REG(dma->M0AR, (usize)memory_addr);
}

void dma_deinit_stream(DMA_Stream_TypeDef* dma);

#ifdef __cplusplus
}
#endif
