#include "app.h"
#include "main.h"
#include "lps22hh.h"

extern TIM_HandleTypeDef htim1;
extern FDCAN_HandleTypeDef hfdcan1;

static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[64] = {0};

void App_Init(void) {
	HAL_Delay(100);
	Lps22hh_Init();
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

//	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
//			0) != HAL_OK) {
//		Error_Handler();
//	}
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_ON;
	tx_header.FDFormat = FDCAN_FD_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

	HAL_TIM_Base_Start_IT(&htim1);
}

void app_process(void) {
	Lps22hh_Update();
	Lps22hh_Data(tx_data);

	tx_header.Identifier = 12;

	tx_header.DataLength = FDCAN_DLC_BYTES_48;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK) {
		Error_Handler();
	}
}

void App_Update(void) {

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	app_process();
}

