#include "app.h"
#include "usbd_cdc_if.h"
#include "lps22hh.h"

void App_Init(void) {
	HAL_Delay(10);
	Lps22hh_Init();
}

void App_Update(void) {
	Lps22hh_Update();
	uint8_t send_buf[LPS22HH_BUF_SIZE];
	Lps22hh_Data(send_buf);
	CDC_Transmit_FS(send_buf, LPS22HH_BUF_SIZE);
	HAL_Delay(10);
}
