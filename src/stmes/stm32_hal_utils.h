#pragma once

#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

__NO_RETURN void crash_on_hal_error(HAL_StatusTypeDef code, const char* file, u32 line);

#define check_hal_error(expr)                           \
  do {                                                  \
    HAL_StatusTypeDef __code__ = (expr);                \
    if (unlikely(__code__ != HAL_OK)) {                 \
      crash_collect_registers();                        \
      crash_on_hal_error(__code__, __FILE__, __LINE__); \
    }                                                   \
  } while (0)

#define BITBAND_ADDR(base, bb_base, addr, bit) \
  ((volatile u32*)(bb_base + 32ul * ((usize)(addr) - (base)) + 4ul * (bit)))
#define SRAM1_BITBAND_ADDR(addr, bit) BITBAND_ADDR(SRAM1_BASE, SRAM1_BB_BASE, addr, bit)
#define PERIPH_BITBAND_ADDR(addr, bit) BITBAND_ADDR(PERIPH_BASE, PERIPH_BB_BASE, addr, bit)

#ifdef __cplusplus
}
#endif
