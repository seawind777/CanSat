/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define LSM6_USE_I2C
#define LIS3_USE_SPI
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M5_C1_Pin GPIO_PIN_13
#define M5_C1_GPIO_Port GPIOC
#define M5_C1_EXTI_IRQn EXTI15_10_IRQn
#define M4_C1_Pin GPIO_PIN_14
#define M4_C1_GPIO_Port GPIOC
#define M4_C1_EXTI_IRQn EXTI15_10_IRQn
#define M4_C2_Pin GPIO_PIN_15
#define M4_C2_GPIO_Port GPIOC
#define LED_ERR_Pin GPIO_PIN_0
#define LED_ERR_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define M2_C1_Pin GPIO_PIN_3
#define M2_C1_GPIO_Port GPIOA
#define M2_C1_EXTI_IRQn EXTI3_IRQn
#define WQ_NSS_Pin GPIO_PIN_4
#define WQ_NSS_GPIO_Port GPIOC
#define M3_C2_Pin GPIO_PIN_0
#define M3_C2_GPIO_Port GPIOB
#define M3_C1_Pin GPIO_PIN_1
#define M3_C1_GPIO_Port GPIOB
#define M3_C1_EXTI_IRQn EXTI1_IRQn
#define M1_C2_Pin GPIO_PIN_12
#define M1_C2_GPIO_Port GPIOB
#define M1_C1_Pin GPIO_PIN_15
#define M1_C1_GPIO_Port GPIOB
#define M1_C1_EXTI_IRQn EXTI15_10_IRQn
#define M6_C2_Pin GPIO_PIN_6
#define M6_C2_GPIO_Port GPIOC
#define M6_C1_Pin GPIO_PIN_7
#define M6_C1_GPIO_Port GPIOC
#define M6_C1_EXTI_IRQn EXTI9_5_IRQn
#define MS_NSS_Pin GPIO_PIN_9
#define MS_NSS_GPIO_Port GPIOA
#define PHOTO_RES_Pin GPIO_PIN_15
#define PHOTO_RES_GPIO_Port GPIOA
#define M5_C2_Pin GPIO_PIN_3
#define M5_C2_GPIO_Port GPIOB
#define RESET_LORA_Pin GPIO_PIN_4
#define RESET_LORA_GPIO_Port GPIOB
#define M2_C2_Pin GPIO_PIN_5
#define M2_C2_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_8
#define LORA_NSS_GPIO_Port GPIOB
#define LIS_NSS_Pin GPIO_PIN_9
#define LIS_NSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
