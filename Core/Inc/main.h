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
#define LED_ERR_Pin GPIO_PIN_0
#define LED_ERR_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define WQ_NSS_Pin GPIO_PIN_4
#define WQ_NSS_GPIO_Port GPIOC
#define MS_NSS_Pin GPIO_PIN_9
#define MS_NSS_GPIO_Port GPIOA
#define PHOTO_RES_Pin GPIO_PIN_15
#define PHOTO_RES_GPIO_Port GPIOA
#define RESET_LORA_Pin GPIO_PIN_4
#define RESET_LORA_GPIO_Port GPIOB
#define SDIO_Detect_Pin GPIO_PIN_5
#define SDIO_Detect_GPIO_Port GPIOB
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
