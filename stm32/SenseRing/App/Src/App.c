/*
 * App.c
 *
 *  Created on: Feb 23, 2025
 *      Author: lionm
 */

#include "App.h"
#include "main.h"
#include "lps22hh.h"


extern FDCAN_HandleTypeDef hfdcan1;

extern SPI_HandleTypeDef hspi1;


FDCAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[32];

struct Lps22hh_Handle sensor1;
struct Lps22hh_Handle sensor1 = {.hspi=&hspi1, .csPort = CS1_GPIO_Port, .csPin = CS1_Pin, .intPin = 0, .addy = 0x1};
struct Lps22hh_Handle sensor2 = {.hspi=&hspi1, .csPort = CS2_GPIO_Port, .csPin = CS2_Pin, .intPin = 0, .addy = 0x2};
struct Lps22hh_Handle sensor3 = {.hspi=&hspi1, .csPort = CS3_GPIO_Port, .csPin = CS3_Pin, .intPin = 0, .addy = 0x3};
struct Lps22hh_Handle sensor4 = {.hspi=&hspi1, .csPort = CS4_GPIO_Port, .csPin = CS4_Pin, .intPin = 0, .addy = 0x4};
struct Lps22hh_Handle sensor5 = {.hspi=&hspi1, .csPort = CS5_GPIO_Port, .csPin = CS5_Pin, .intPin = 0, .addy = 0x5};
struct Lps22hh_Handle sensor6 = {.hspi=&hspi1, .csPort = CS6_GPIO_Port, .csPin = CS6_Pin, .intPin = 0, .addy = 0x6};


void compileData(struct Lps22hh_Handle* sensor, uint8_t* Arr);

void App_Init(void){

	sensor1.hspi = &hspi1;//10Mhz max
	sensor1.csPort = GPIOA;
	sensor1.csPin = GPIO_PIN_3;
	sensor1.intPin = 0;

	FDCAN_Config();
	TxData[0] = 100;
	TxData[1] = 50;
	TxData[2] = 90;
	TxData[3] = 120;
	TxData[4] = 200;
	TxData[5] = 150;
	TxData[6] = 60;
	TxData[7] = 70;

	Lps22hh_Init(&sensor1);
	Lps22hh_Init(&sensor1);
	Lps22hh_Init(&sensor2);
	Lps22hh_Init(&sensor3);
	Lps22hh_Init(&sensor4);
	Lps22hh_Init(&sensor5);
	Lps22hh_Init(&sensor6);


}

void App_Update(void){
	Lps22hh_ExtHandler(&sensor1);
	Lps22hh_ExtHandler(&sensor2);
	Lps22hh_ExtHandler(&sensor3);
	Lps22hh_ExtHandler(&sensor4);
	Lps22hh_ExtHandler(&sensor5);
	Lps22hh_ExtHandler(&sensor6);

//	TxData[0] = *sensor1.data;
//	TxData[1] = *(sensor1.data+1);
//	TxData[2] = *(sensor1.data+2);
//	TxData[3] = *(sensor1.data+3);
//	TxData[4] = *(sensor1.data+4);
	TxData[5] = 0x1;
	TxData[6] = 0xff;
	TxData[7]++;



	HAL_Delay(100);
	TxData[0] = 0xff;
//	TxData[1] = 0xff;

	compileData(&sensor1,TxData);
	compileData(&sensor2,TxData);
	compileData(&sensor3,TxData);
	compileData(&sensor4,TxData);
	compileData(&sensor5,TxData);
	compileData(&sensor6,TxData);

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) Error_Handler();

	HAL_Delay(10);


}

void compileData(struct Lps22hh_Handle* sensor, uint8_t* Arr){
	for(int i = 0; i < 3;i++){
		Arr[((sensor->addy) * 3) + i] = sensor->data[i];
	}
}



void FDCAN_Config(void){
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x7FF;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  TxHeader.DataLength = FDCAN_DLC_BYTES_32;

}







