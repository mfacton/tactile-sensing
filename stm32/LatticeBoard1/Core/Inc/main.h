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
#define CSA1_Pin GPIO_PIN_2
#define CSA1_GPIO_Port GPIOE
#define CSC11_Pin GPIO_PIN_3
#define CSC11_GPIO_Port GPIOE
#define CSC15_Pin GPIO_PIN_4
#define CSC15_GPIO_Port GPIOC
#define CSC14_Pin GPIO_PIN_5
#define CSC14_GPIO_Port GPIOC
#define CSC13_Pin GPIO_PIN_0
#define CSC13_GPIO_Port GPIOB
#define CSC12_Pin GPIO_PIN_2
#define CSC12_GPIO_Port GPIOB
#define CSC3_Pin GPIO_PIN_8
#define CSC3_GPIO_Port GPIOE
#define CSC2_Pin GPIO_PIN_9
#define CSC2_GPIO_Port GPIOE
#define CSC1_Pin GPIO_PIN_10
#define CSC1_GPIO_Port GPIOE
#define CSB15_Pin GPIO_PIN_11
#define CSB15_GPIO_Port GPIOE
#define CSB14_Pin GPIO_PIN_12
#define CSB14_GPIO_Port GPIOE
#define CSB13_Pin GPIO_PIN_13
#define CSB13_GPIO_Port GPIOE
#define CSB12_Pin GPIO_PIN_14
#define CSB12_GPIO_Port GPIOE
#define CSB11_Pin GPIO_PIN_15
#define CSB11_GPIO_Port GPIOE
#define CSB10_Pin GPIO_PIN_12
#define CSB10_GPIO_Port GPIOB
#define CSB9_Pin GPIO_PIN_13
#define CSB9_GPIO_Port GPIOB
#define CSC8_Pin GPIO_PIN_14
#define CSC8_GPIO_Port GPIOB
#define CSB8_Pin GPIO_PIN_15
#define CSB8_GPIO_Port GPIOB
#define CSC7_Pin GPIO_PIN_8
#define CSC7_GPIO_Port GPIOD
#define CSB7_Pin GPIO_PIN_9
#define CSB7_GPIO_Port GPIOD
#define CSC6_Pin GPIO_PIN_10
#define CSC6_GPIO_Port GPIOD
#define CSB6_Pin GPIO_PIN_11
#define CSB6_GPIO_Port GPIOD
#define CSC5_Pin GPIO_PIN_12
#define CSC5_GPIO_Port GPIOD
#define CSB5_Pin GPIO_PIN_13
#define CSB5_GPIO_Port GPIOD
#define CSC4_Pin GPIO_PIN_14
#define CSC4_GPIO_Port GPIOD
#define CSB4_Pin GPIO_PIN_15
#define CSB4_GPIO_Port GPIOD
#define CSB3_Pin GPIO_PIN_7
#define CSB3_GPIO_Port GPIOC
#define CSB2_Pin GPIO_PIN_8
#define CSB2_GPIO_Port GPIOC
#define CSB1_Pin GPIO_PIN_9
#define CSB1_GPIO_Port GPIOC
#define CSA15_Pin GPIO_PIN_8
#define CSA15_GPIO_Port GPIOA
#define CSA14_Pin GPIO_PIN_10
#define CSA14_GPIO_Port GPIOA
#define CSA8_Pin GPIO_PIN_0
#define CSA8_GPIO_Port GPIOD
#define CSA9_Pin GPIO_PIN_1
#define CSA9_GPIO_Port GPIOD
#define CSA10_Pin GPIO_PIN_2
#define CSA10_GPIO_Port GPIOD
#define CSA7_Pin GPIO_PIN_3
#define CSA7_GPIO_Port GPIOD
#define CSA6_Pin GPIO_PIN_4
#define CSA6_GPIO_Port GPIOD
#define CSA11_Pin GPIO_PIN_5
#define CSA11_GPIO_Port GPIOD
#define CSA12_Pin GPIO_PIN_6
#define CSA12_GPIO_Port GPIOD
#define CSA13_Pin GPIO_PIN_7
#define CSA13_GPIO_Port GPIOD
#define CSC9_Pin GPIO_PIN_4
#define CSC9_GPIO_Port GPIOB
#define CSC10_Pin GPIO_PIN_5
#define CSC10_GPIO_Port GPIOB
#define CSA5_Pin GPIO_PIN_8
#define CSA5_GPIO_Port GPIOB
#define CSA4_Pin GPIO_PIN_9
#define CSA4_GPIO_Port GPIOB
#define CSA3_Pin GPIO_PIN_0
#define CSA3_GPIO_Port GPIOE
#define CSA2_Pin GPIO_PIN_1
#define CSA2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
