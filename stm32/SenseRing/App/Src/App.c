/*
 * App.c
 *
 *  Created on: Feb 23, 2025
 *      Author: lionm
 */

#include "App.h"
#include "main.h"
//#include "lps22hh.h"


extern FDCAN_HandleTypeDef hfdcan1;

extern SPI_HandleTypeDef hspi1;


FDCAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];

//struct Lps22hh_Handle sensor1 = {.hspi=&hspi1, .csPort = CS1_GPIO_Port, .csPin = CS1_Pin};



void App_Init(void){


	FDCAN_Config();
	TxData[0] = 100;
	TxData[1] = 50;
	TxData[2] = 90;
	TxData[3] = 120;
	TxData[4] = 200;
	TxData[5] = 150;
	TxData[6] = 60;
	TxData[7] = 70;

//	Lps22hh_Init(&sensor1);


}

void App_Update(void){
//	Lps22hh_ExtHandler(&sensor1);

//	TxData[0] = *sensor1.data;
//	TxData[1] = *(sensor1.data+1);
//	TxData[2] = *(sensor1.data+2);
//	TxData[3] = *(sensor1.data+3);
//	TxData[4] = *(sensor1.data+4);
	TxData[5] = 0x1;
	TxData[6] = 0xff;
	TxData[7]++;

	HAL_Delay(100);


	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) Error_Handler();


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
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  TxHeader.DataLength = FDCAN_DLC_BYTES_8;

}







