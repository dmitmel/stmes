/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLTN_LED_Pin GPIO_PIN_13
#define BLTN_LED_GPIO_Port GPIOC
#define BLTN_KEY_Pin GPIO_PIN_0
#define BLTN_KEY_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_1
#define SDIO_CD_GPIO_Port GPIOA
#define VGA_HSYNC_Pin GPIO_PIN_2
#define VGA_HSYNC_GPIO_Port GPIOA
#define VGA_VSYNC_Pin GPIO_PIN_3
#define VGA_VSYNC_GPIO_Port GPIOA
#define SDIO_CMD_Pin GPIO_PIN_6
#define SDIO_CMD_GPIO_Port GPIOA
#define VGA_PIXEL1_Pin GPIO_PIN_0
#define VGA_PIXEL1_GPIO_Port GPIOB
#define VGA_PIXEL2_Pin GPIO_PIN_1
#define VGA_PIXEL2_GPIO_Port GPIOB
#define VGA_PIXEL3_Pin GPIO_PIN_2
#define VGA_PIXEL3_GPIO_Port GPIOB
#define VGA_PIXEL9_Pin GPIO_PIN_10
#define VGA_PIXEL9_GPIO_Port GPIOB
#define VGA_PIXEL10_Pin GPIO_PIN_12
#define VGA_PIXEL10_GPIO_Port GPIOB
#define VGA_PIXEL11_Pin GPIO_PIN_13
#define VGA_PIXEL11_GPIO_Port GPIOB
#define VGA_PIXEL12_Pin GPIO_PIN_14
#define VGA_PIXEL12_GPIO_Port GPIOB
#define SDIO_CLK_Pin GPIO_PIN_15
#define SDIO_CLK_GPIO_Port GPIOB
#define SDIO_D1_Pin GPIO_PIN_8
#define SDIO_D1_GPIO_Port GPIOA
#define SDIO_D2_Pin GPIO_PIN_9
#define SDIO_D2_GPIO_Port GPIOA
#define VGA_PIXEL4_Pin GPIO_PIN_3
#define VGA_PIXEL4_GPIO_Port GPIOB
#define SDIO_D0_Pin GPIO_PIN_4
#define SDIO_D0_GPIO_Port GPIOB
#define SDIO_D3_Pin GPIO_PIN_5
#define SDIO_D3_GPIO_Port GPIOB
#define VGA_PIXEL5_Pin GPIO_PIN_6
#define VGA_PIXEL5_GPIO_Port GPIOB
#define VGA_PIXEL6_Pin GPIO_PIN_7
#define VGA_PIXEL6_GPIO_Port GPIOB
#define VGA_PIXEL7_Pin GPIO_PIN_8
#define VGA_PIXEL7_GPIO_Port GPIOB
#define VGA_PIXEL8_Pin GPIO_PIN_9
#define VGA_PIXEL8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
