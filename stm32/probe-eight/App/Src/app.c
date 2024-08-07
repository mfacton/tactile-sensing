#include "app.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "lps22hh.h"

void App_Init(void) {
	HAL_Delay(100);
	Lps22hh_Init();
}

void App_Update(void) {
	Lps22hh_Update();
	CDC_Transmit_FS(Lps22hh_Data(), LPS22HH_BUF_SIZE);
	HAL_Delay(5);
}
