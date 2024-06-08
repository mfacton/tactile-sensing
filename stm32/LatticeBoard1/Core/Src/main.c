/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LPS22HB_ADDRESS            0xBA

#define LPS22HB_DEVICE_NAME        0xB1

#define LPS22HB_REG_WHO_AM_I       0x0F
#define LPS22HB_REG_CTRL_REG1      0x10
#define LPS22HB_REG_CTRL_REG2      0x11
#define LPS22HB_REG_CTRL_REG3      0x12
#define LPS22HB_REG_RES_CONF       0x1A
#define LPS22HB_REG_STATUS         0x27
#define LPS22HB_REG_PRESS_OUT_XL   0x28
#define LPS22HB_REG_PRESS_OUT_L    0x29
#define LPS22HB_REG_PRESS_OUT_H    0x2A
#define LPS22HB_REG_TEMP_OUT_L     0x2B
#define LPS22HB_REG_TEMP_OUT_H     0x2C
#define LPS22HB_REG_LPFP_RES       0x33

SPI_HandleTypeDef *spiBusses[3] = { &hspi3, &hspi2, &hspi1 };
GPIO_TypeDef *chipSelectPorts[3][15] = { { CSA1_GPIO_Port, CSA2_GPIO_Port,

CSA3_GPIO_Port, CSA4_GPIO_Port, CSA5_GPIO_Port, CSA6_GPIO_Port, CSA7_GPIO_Port,
		CSA8_GPIO_Port, CSA9_GPIO_Port, CSA10_GPIO_Port, CSA11_GPIO_Port,
		CSA12_GPIO_Port, CSA13_GPIO_Port, CSA14_GPIO_Port, CSA15_GPIO_Port }, {
		CSB1_GPIO_Port, CSB2_GPIO_Port, CSB3_GPIO_Port,

		CSB4_GPIO_Port, CSB5_GPIO_Port, CSB6_GPIO_Port, CSB7_GPIO_Port,
		CSB8_GPIO_Port, CSB9_GPIO_Port, CSB10_GPIO_Port, CSB11_GPIO_Port,
		CSB12_GPIO_Port, CSB13_GPIO_Port, CSB14_GPIO_Port, CSB15_GPIO_Port }, {
		CSC1_GPIO_Port, CSC2_GPIO_Port, CSC3_GPIO_Port, CSC4_GPIO_Port,
		CSC5_GPIO_Port, CSC6_GPIO_Port, CSC7_GPIO_Port, CSC8_GPIO_Port,
		CSC9_GPIO_Port, CSC10_GPIO_Port, CSC11_GPIO_Port, CSC12_GPIO_Port,
		CSC13_GPIO_Port, CSC14_GPIO_Port, CSC15_GPIO_Port }, };
uint16_t chipSelectsPins[3][15] = { { CSA1_Pin, CSA2_Pin, CSA3_Pin, CSA4_Pin,
		CSA5_Pin, CSA6_Pin, CSA7_Pin, CSA8_Pin, CSA9_Pin, CSA10_Pin, CSA11_Pin,
		CSA12_Pin, CSA13_Pin, CSA14_Pin, CSA15_Pin }, { CSB1_Pin, CSB2_Pin,
		CSB3_Pin, CSB4_Pin, CSB5_Pin, CSB6_Pin, CSB7_Pin, CSB8_Pin, CSB9_Pin,
		CSB10_Pin, CSB11_Pin, CSB12_Pin, CSB13_Pin, CSB14_Pin, CSB15_Pin }, {
		CSC1_Pin, CSC2_Pin, CSC3_Pin, CSC4_Pin, CSC5_Pin, CSC6_Pin, CSC7_Pin,
		CSC8_Pin, CSC9_Pin, CSC10_Pin, CSC11_Pin, CSC12_Pin, CSC13_Pin,
		CSC14_Pin, CSC15_Pin }, };

void LPS22HB_WriteReg(uint8_t bus, uint16_t sel, uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(chipSelectPorts[bus][sel], chipSelectsPins[bus][sel],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiBusses[bus], &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(spiBusses[bus], &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(chipSelectPorts[bus][sel], chipSelectsPins[bus][sel],
			GPIO_PIN_SET);
}

uint8_t LPS22HB_ReadReg(uint8_t bus, uint16_t sel, uint8_t reg) {
	uint8_t data = 0;
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(chipSelectPorts[bus][sel], chipSelectsPins[bus][sel],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiBusses[bus], &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spiBusses[bus], &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(chipSelectPorts[bus][sel], chipSelectsPins[bus][sel],
			GPIO_PIN_SET);
	return data;
}

void LPS22HB_ReadRegs(uint8_t bus, uint16_t sel, uint8_t reg, uint8_t *output,
		uint16_t len) {
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(chipSelectPorts[bus][sel], chipSelectsPins[bus][sel],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(spiBusses[bus], &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spiBusses[bus], output, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(chipSelectPorts[bus][sel], chipSelectsPins[bus][sel],
			GPIO_PIN_SET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(1000);

	for (uint8_t b = 0; b < 3; b++) {
		for (uint8_t cs = 0; cs < 15; cs++) {
			uint8_t who_am_i = LPS22HB_ReadReg(b, cs, LPS22HB_REG_WHO_AM_I);
			if (who_am_i != LPS22HB_DEVICE_NAME) {
				while (1) {
				}
			}
			//reboot memory content
			//			LPS22HB_WriteReg(b, cs, LPS22HB_REG_CTRL_REG1, 0b10000000);
			//			//wait for reboot
			//			while (LPS22HB_ReadReg(b, cs, LPS22HB_REG_CTRL_REG1) >> 7) {
			//				HAL_Delay(10);
			//			}
			//			LPS22HB_WriteReg(b, cs, LPS22HB_REG_CTRL_REG1, 0b00000010);
			//			while ((LPS22HB_ReadReg(b, cs, LPS22HB_REG_CTRL_REG1) >> 1) & 1) {
			//				HAL_Delay(1);
			//			}
			//disable I2C
			LPS22HB_WriteReg(b, cs, LPS22HB_REG_CTRL_REG1, 0b00000100);

			//configure LPF to /2
			LPS22HB_WriteReg(b, cs, LPS22HB_REG_CTRL_REG1, 0b01011000);
			//reset filter
			LPS22HB_ReadReg(b, cs, LPS22HB_REG_LPFP_RES);
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		uint8_t data[225];
		for (uint8_t b = 0; b < 3; b++) {
			for (uint8_t cs = 0; cs < 15; cs++) {
				uint16_t index = 5 * (b * 15 + cs);
				LPS22HB_ReadRegs(b, cs, LPS22HB_REG_PRESS_OUT_XL, data + index,
						3);
				LPS22HB_ReadRegs(b, cs, LPS22HB_REG_TEMP_OUT_L,
						data + index + 3, 2);
			}
		}

		CDC_Transmit_FS(data, 225);

		HAL_Delay(26);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CSA1_Pin|CSC11_Pin|CSC3_Pin|CSC2_Pin
                          |CSC1_Pin|CSB15_Pin|CSB14_Pin|CSB13_Pin
                          |CSB12_Pin|CSB11_Pin|CSA3_Pin|CSA2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CSC15_Pin|CSC14_Pin|CSB3_Pin|CSB2_Pin
                          |CSB1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSC13_Pin|CSC12_Pin|CSB10_Pin|CSB9_Pin
                          |CSC8_Pin|CSB8_Pin|CSC9_Pin|CSC10_Pin
                          |CSA5_Pin|CSA4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CSC7_Pin|CSB7_Pin|CSC6_Pin|CSB6_Pin
                          |CSC5_Pin|CSB5_Pin|CSC4_Pin|CSB4_Pin
                          |CSA8_Pin|CSA9_Pin|CSA10_Pin|CSA7_Pin
                          |CSA6_Pin|CSA11_Pin|CSA12_Pin|CSA13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSA15_Pin|CSA14_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CSA1_Pin CSC11_Pin CSC3_Pin CSC2_Pin
                           CSC1_Pin CSB15_Pin CSB14_Pin CSB13_Pin
                           CSB12_Pin CSB11_Pin CSA3_Pin CSA2_Pin */
  GPIO_InitStruct.Pin = CSA1_Pin|CSC11_Pin|CSC3_Pin|CSC2_Pin
                          |CSC1_Pin|CSB15_Pin|CSB14_Pin|CSB13_Pin
                          |CSB12_Pin|CSB11_Pin|CSA3_Pin|CSA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CSC15_Pin CSC14_Pin CSB3_Pin CSB2_Pin
                           CSB1_Pin */
  GPIO_InitStruct.Pin = CSC15_Pin|CSC14_Pin|CSB3_Pin|CSB2_Pin
                          |CSB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CSC13_Pin CSC12_Pin CSB10_Pin CSB9_Pin
                           CSC8_Pin CSB8_Pin CSC9_Pin CSC10_Pin
                           CSA5_Pin CSA4_Pin */
  GPIO_InitStruct.Pin = CSC13_Pin|CSC12_Pin|CSB10_Pin|CSB9_Pin
                          |CSC8_Pin|CSB8_Pin|CSC9_Pin|CSC10_Pin
                          |CSA5_Pin|CSA4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CSC7_Pin CSB7_Pin CSC6_Pin CSB6_Pin
                           CSC5_Pin CSB5_Pin CSC4_Pin CSB4_Pin
                           CSA8_Pin CSA9_Pin CSA10_Pin CSA7_Pin
                           CSA6_Pin CSA11_Pin CSA12_Pin CSA13_Pin */
  GPIO_InitStruct.Pin = CSC7_Pin|CSB7_Pin|CSC6_Pin|CSB6_Pin
                          |CSC5_Pin|CSB5_Pin|CSC4_Pin|CSB4_Pin
                          |CSA8_Pin|CSA9_Pin|CSA10_Pin|CSA7_Pin
                          |CSA6_Pin|CSA11_Pin|CSA12_Pin|CSA13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CSA15_Pin CSA14_Pin */
  GPIO_InitStruct.Pin = CSA15_Pin|CSA14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
