/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define DR1_Pin GPIO_PIN_0
#define DR1_GPIO_Port GPIOA
#define DR2_Pin GPIO_PIN_1
#define DR2_GPIO_Port GPIOA
#define DR3_Pin GPIO_PIN_2
#define DR3_GPIO_Port GPIOA
#define DR6_Pin GPIO_PIN_3
#define DR6_GPIO_Port GPIOA
#define DR5_Pin GPIO_PIN_4
#define DR5_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_0
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_1
#define CS2_GPIO_Port GPIOB
#define CS3_Pin GPIO_PIN_2
#define CS3_GPIO_Port GPIOB
#define DR4_Pin GPIO_PIN_10
#define DR4_GPIO_Port GPIOB
#define CS4_Pin GPIO_PIN_12
#define CS4_GPIO_Port GPIOB
#define CS5_Pin GPIO_PIN_13
#define CS5_GPIO_Port GPIOB
#define CS6_Pin GPIO_PIN_14
#define CS6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
