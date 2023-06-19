#pragma once

#include "stmes/kernel/crash.h"
#include "stmes/utils.h"
#include <ff.h>
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

__NO_RETURN void crash_on_fs_error(FRESULT code, const char* file, u32 line);

#define check_fs_error(expr)                       \
  do {                                             \
    FRESULT code = (expr);                         \
    if (unlikely(code != FR_OK)) {                 \
      crash_collect_registers();                   \
      crash_on_fs_error(code, __FILE__, __LINE__); \
    }                                              \
  } while (0)

#ifdef __cplusplus
}
#endif
