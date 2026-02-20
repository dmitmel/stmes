#include "stmes/stm32_hal_utils.h"
#include "stmes/kernel/crash.h"
#include <printf.h>

__NO_RETURN void crash_on_hal_error(HAL_StatusTypeDef code, const char* file, u32 line) {
  const char* name;
  switch (code) {
    case HAL_OK: name = "OK"; break;
    case HAL_ERROR: name = "ERROR"; break;
    case HAL_BUSY: name = "BUSY"; break;
    case HAL_TIMEOUT: name = "TIMEOUT"; break;
    default: name = "UNKNOWN";
  }
  char msg[32];
  snprintf(msg, sizeof(msg), "0x%02" PRIX32 "/HAL_%s", (u32)code, name);
  crash(msg, file, line);
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line) {
  crash("STM32 HAL assertion failed", (const char*)file, line);
}
#endif
