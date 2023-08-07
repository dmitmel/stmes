#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void prestart(void);
int main(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif
