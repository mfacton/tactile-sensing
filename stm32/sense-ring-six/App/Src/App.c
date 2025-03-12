/*
 * App.c
 *
 *  Created on: Feb 23, 2025
 *      Author: lionm
 */

#include "App.h"
#include "main.h"
#include "lps22hh_reg.h"

extern FDCAN_HandleTypeDef hfdcan1;

extern SPI_HandleTypeDef hspi1;


FDCAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];



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

}

void App_Update(void){

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
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

}













