#pragma once

#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct Task main_task;
extern u8 main_task_stack[1024] __ALIGNED(8);

void prestart(void);
int main(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif
