#pragma once

#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <ff.h>
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

__NO_RETURN void crash_on_fs_error(FRESULT code, const char* file, u32 line);

#define check_fs_error(expr)                           \
  do {                                                 \
    FRESULT __code__ = (expr);                         \
    if (unlikely(__code__ != FR_OK)) {                 \
      crash_collect_registers();                       \
      crash_on_fs_error(__code__, __FILE__, __LINE__); \
    }                                                  \
  } while (0)

#ifdef __cplusplus
}
#endif
