#include "stmes/interrupts.h"
#include "stmes/gpio.h"
#include "stmes/main.h"
#include "stmes/timers.h"
#include "stmes/utils.h"
#include <stm32f4xx_hal.h>

void NMI_Handler(void) {
  while (1) {}
}

void HardFault_Handler(void) {
  while (1) {}
}

void MemManage_Handler(void) {
  while (1) {}
}

void BusFault_Handler(void) {
  while (1) {}
}

void UsageFault_Handler(void) {
  while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
  HAL_IncTick();
}

void TIM2_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim4);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM3) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      GPIO_RESET_PIN(VGA_PIXEL_GPIO_Port, VGA_PIXEL_Pin);
      inside_frame_horizontal = false;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      inside_frame_horizontal = true;
    }
  } else if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      GPIO_RESET_PIN(VGA_PIXEL_GPIO_Port, VGA_PIXEL_Pin);
      inside_frame_vertical = false;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      inside_frame_vertical = true;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM4) {
    frame_counter++;
  }
}
