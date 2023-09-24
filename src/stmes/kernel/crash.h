#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void crash_collect_registers(void);

__NO_RETURN void crash(const char* message, const char* src_file, u32 src_line);

#define CRASH(message) (crash_collect_registers(), crash((message), __FILE__, __LINE__))

#define ASSERT(expr) (unlikely(!(expr)) ? CRASH("Assertion failed") : (void)0)

__NO_RETURN void crash_on_hal_error(HAL_StatusTypeDef code, const char* file, u32 line);

#define check_hal_error(expr)                           \
  do {                                                  \
    HAL_StatusTypeDef __code__ = (expr);                \
    if (unlikely(__code__ != HAL_OK)) {                 \
      crash_collect_registers();                        \
      crash_on_hal_error(__code__, __FILE__, __LINE__); \
    }                                                   \
  } while (0)

__NO_RETURN void enter_crash_screen(void);

void crash_init_hard_faults(void);

#ifdef __cplusplus
}
#endif
