#include "lps22hh.h"
#include "string.h"
#include "main.h"

extern SPI_HandleTypeDef LPS22HH_HANDLE;

static GPIO_TypeDef *chipSelectPorts[7] = { CS1_GPIO_Port, CS2_GPIO_Port,
		CS3_GPIO_Port, CS4_GPIO_Port, CS5_GPIO_Port, CS6_GPIO_Port,
		CS7_GPIO_Port };

static uint16_t chipSelectsPins[7] = { CS1_Pin, CS2_Pin, CS3_Pin, CS4_Pin,
		CS5_Pin, CS6_Pin, CS7_Pin};

static uint8_t data_buf[LPS22HH_BUF_SIZE];

void lps22hh_write_register(enum Sensor sensor, uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(chipSelectPorts[sensor], chipSelectsPins[sensor], 0);
	uint8_t buf[2] = {reg, data};
	HAL_SPI_Transmit(&LPS22HH_HANDLE, buf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(chipSelectPorts[sensor], chipSelectsPins[sensor], 1);
}

uint8_t lps22hh_read_register(enum Sensor sensor, uint8_t reg) {
	uint8_t data = 0;
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(chipSelectPorts[sensor], chipSelectsPins[sensor], 0);
	HAL_SPI_Transmit(&LPS22HH_HANDLE, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LPS22HH_HANDLE, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(chipSelectPorts[sensor], chipSelectsPins[sensor], 1);
	return data;
}

void lps22hh_read_registers(enum Sensor sensor, uint8_t reg, uint8_t *output, uint16_t len) {
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(chipSelectPorts[sensor], chipSelectsPins[sensor], 0);
	HAL_SPI_Transmit(&LPS22HH_HANDLE, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&LPS22HH_HANDLE, output, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(chipSelectPorts[sensor], chipSelectsPins[sensor], 1);
}

void Lps22hh_Init(void) {
	for (uint8_t cs = 0; cs < LPS22HH_COUNT; cs++) {
		uint8_t who_am_i = lps22hh_read_register(cs, LPS22HH_REG_WHO_AM_I);
		if (who_am_i != LPS22HH_DEVICE_NAME) {
//			uint8_t buf[40] = {0};
//			memset(buf, cs, 40);
			while (1) {
//				CDC_Transmit_FS(buf, 40);
//				HAL_Delay(5);
			}
		}

		//configure ODR 200hz
		//enable low pass filter at bandwidth odr/9
		lps22hh_write_register(cs, LPS22HH_REG_CTRL_REG1, 0b01111100);
	}
}

void Lps22hh_Update(void) {
	for (uint8_t cs = 0; cs < LPS22HH_COUNT; cs++) {
		uint16_t index = 5 * cs;
		lps22hh_read_registers(cs, LPS22HH_REG_PRESS_OUT_XL, data_buf + index, 3);
		lps22hh_read_registers(cs, LPS22HH_REG_TEMP_OUT_L, data_buf + index + 3, 2);
	}
}

void Lps22hh_Data(uint8_t *buf) {
	memcpy(buf, data_buf, LPS22HH_BUF_SIZE);
}
