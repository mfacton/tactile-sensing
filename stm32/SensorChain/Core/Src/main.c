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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_TypeDef *selectPorts[6] = {CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port, CS4_GPIO_Port, CS5_GPIO_Port, CS6_GPIO_Port};
uint16_t selectPins[6] = {CS1_Pin, CS2_Pin, CS3_Pin, CS4_Pin, CS5_Pin, CS6_Pin};

GPIO_TypeDef *dataReadyPorts[6] = {DR1_GPIO_Port, DR2_GPIO_Port, DR3_GPIO_Port, DR4_GPIO_Port, DR5_GPIO_Port, DR6_GPIO_Port};
uint16_t dataReadyPins[6] = {DR1_Pin, DR2_Pin, DR3_Pin, DR4_Pin, DR5_Pin, DR6_Pin};

void ADS1220_Reset(uint8_t bus)
{
	uint8_t value = 0x06;
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_SET);
}

void ADS1220_StartConversion(uint8_t bus)
{
	uint8_t value = 0x08;
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_SET);
}

void ADS1220_ReadMeasurment(uint8_t bus, uint8_t *output)
{
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, output, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_SET);
}

void ADS1220_WriteReg(uint8_t bus, uint8_t reg, uint8_t value)
{
	uint8_t regRW = 0x40 | (reg<<2);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_SET);
}

uint8_t ADS1220_ReadReg(uint8_t bus, uint8_t reg) {
	uint8_t data = 0;
	uint8_t regRW = 0x20 | (reg<<2);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(selectPorts[bus], selectPins[bus],
			GPIO_PIN_SET);
	return data;
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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(50);

  for (int i = 0; i < 6; i++)
  {
	  ADS1220_Reset(i);
  }

  HAL_Delay(10);

  for (int i = 0; i < 6; i++)
  {
	  ADS1220_WriteReg(i, 0, 0b10000001);
	  ADS1220_WriteReg(i, 1, 0b11000100);
	  ADS1220_WriteReg(i, 2, 0b11000000);
	  ADS1220_WriteReg(i, 3, 0b00000000);
  }

  HAL_Delay(10);
  for (int i = 0; i < 6; i++)
  {
	  //set mux to 1
	  ADS1220_StartConversion(i);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t data[72] = {0};
	  for (int c = 0; c < 4; c++)
	  {
		  HAL_Delay(1);
		  for (int i = 0; i < 6; i++)
		  {
			  if (i == 1) {
				  continue;
			  }
			  ADS1220_StartConversion(i);
		  }
		  for (int i = 0; i < 6; i++)
		  {
			  if (i == 1) {
				  continue;
			  }
//			  ADS1220_StartConversion(i);
			  while (HAL_GPIO_ReadPin(dataReadyPorts[i], dataReadyPins[i])) {}
			  ADS1220_ReadMeasurment(i, data+(c*18+i*3));
			  ADS1220_WriteReg(i, 0, 0b10000001 | (((c+1)&3)<<4));
		  }
	  }
	  CDC_Transmit_FS(data, 72);
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS2_Pin|CS3_Pin|CS4_Pin
                          |CS5_Pin|CS6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DR1_Pin DR2_Pin DR3_Pin DR6_Pin
                           DR5_Pin */
  GPIO_InitStruct.Pin = DR1_Pin|DR2_Pin|DR3_Pin|DR6_Pin
                          |DR5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS2_Pin CS3_Pin CS4_Pin
                           CS5_Pin CS6_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin|CS3_Pin|CS4_Pin
                          |CS5_Pin|CS6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DR4_Pin */
  GPIO_InitStruct.Pin = DR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DR4_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
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
