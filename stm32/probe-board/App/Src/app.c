#include "app.h"
#include "usbd_cdc_if.h"
#include "lps22hh.h"

extern TIM_HandleTypeDef htim1;

void App_Init(void) {
	HAL_Delay(100);
	Lps22hh_Init();
	HAL_TIM_Base_Start_IT(&htim1);
}

void app_process(void) {
	Lps22hh_Update();
	uint8_t send_buf[LPS22HH_BUF_SIZE];
	Lps22hh_Data(send_buf);
	CDC_Transmit_FS(send_buf, LPS22HH_BUF_SIZE);
}

void App_Update(void) {
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	app_process();
}

