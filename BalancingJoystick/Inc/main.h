/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define KEY_K5_Pin GPIO_PIN_3
#define KEY_K5_GPIO_Port GPIOE
#define ICM42688_CS_Pin GPIO_PIN_4
#define ICM42688_CS_GPIO_Port GPIOE
#define ICM42688_INT1_Pin GPIO_PIN_5
#define ICM42688_INT1_GPIO_Port GPIOE
#define RUN_LED_Pin GPIO_PIN_1
#define RUN_LED_GPIO_Port GPIOA
#define KEY_K6_Pin GPIO_PIN_5
#define KEY_K6_GPIO_Port GPIOC
#define KEY_K1_Pin GPIO_PIN_7
#define KEY_K1_GPIO_Port GPIOE
#define KEY_K2_Pin GPIO_PIN_9
#define KEY_K2_GPIO_Port GPIOE
#define KEY_K3_Pin GPIO_PIN_11
#define KEY_K3_GPIO_Port GPIOE
#define KEY_K4_Pin GPIO_PIN_13
#define KEY_K4_GPIO_Port GPIOE
#define X_EN_Pin GPIO_PIN_8
#define X_EN_GPIO_Port GPIOB
#define X_DIR_Pin GPIO_PIN_0
#define X_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
