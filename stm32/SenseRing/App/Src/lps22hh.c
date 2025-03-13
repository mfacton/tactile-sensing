#include "lps22hh.h"
#include "string.h"
#include "stdlib.h"

static void lps22hh_write_register(struct Lps22hh_Handle* handle, uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
	HAL_SPI_Transmit(handle->hspi, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(handle->hspi, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
}

static uint8_t lps22hh_read_register(struct Lps22hh_Handle* handle, uint8_t reg) {
	uint8_t data = 0;
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
	HAL_SPI_Transmit(handle->hspi, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(handle->hspi, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
	return data;
}

static void lps22hh_read_registers(struct Lps22hh_Handle* handle, uint8_t reg, uint8_t *output, uint16_t len) {
	uint8_t regRW = 0x80 | reg;
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, 0);
	HAL_SPI_Transmit(handle->hspi, &regRW, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(handle->hspi, output, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(handle->csPort, handle->csPin, 1);
}

void Lps22hh_Init(struct Lps22hh_Handle* handle) {
	handle->data = malloc(5);
	memset(handle->data, 0, 5);

	const uint8_t whoami = lps22hh_read_register(handle, LPS22HH_REG_WHO_AM_I);
	if (whoami != LPS22HH_ID) {
		Error_Handler();
	}
	Lps22hh_Reset(handle);

	Lps22hh_SetODR(handle, Lps22hh_Odr10);
	Lps22hh_SetFilter(handle, Lps22hh_Filt20);
	Lps22hh_SetInterrupt(handle, 1);

	handle->init = 1;

	Lps22hh_ExtHandler(handle);
}

uint8_t Lps22hh_ExtFlag(struct Lps22hh_Handle* handle, uint16_t pin) {
	return handle->intPin == pin;
}

void Lps22hh_ExtHandler(struct Lps22hh_Handle* handle) {
	if (handle->init) {
		lps22hh_read_registers(handle, LPS22HH_REG_PRESS_XL, handle->data, 5);
	}
}

void Lps22hh_Reset(struct Lps22hh_Handle* handle) {
	const uint8_t val = lps22hh_read_register(handle, LPS22HH_REG_CTRL_REG2);
	lps22hh_write_register(handle, LPS22HH_REG_CTRL_REG2, val | 0x04);
	while (1) {
		HAL_Delay(1);
		if (!((lps22hh_read_register(handle, LPS22HH_REG_CTRL_REG2) >> 2) & 1)) {
			break;
		}
	}
}

void Lps22hh_SetODR(struct Lps22hh_Handle* handle, enum Lps22hh_Odr odr) {
	uint8_t val = lps22hh_read_register(handle, LPS22HH_REG_CTRL_REG1);
	val = (val & 0x0f) | (odr << 4);
	lps22hh_write_register(handle, LPS22HH_REG_CTRL_REG1, val);
}

void Lps22hh_SetFilter(struct Lps22hh_Handle* handle, enum Lps22hh_Filt filt) {
	uint8_t val = lps22hh_read_register(handle, LPS22HH_REG_CTRL_REG1);
	val = (val & 0x73) | (filt << 2);
	lps22hh_write_register(handle, LPS22HH_REG_CTRL_REG1, val);
}

void Lps22hh_SetInterrupt(struct Lps22hh_Handle* handle, uint8_t state) {
	uint8_t val = lps22hh_read_register(handle, LPS22HH_REG_CTRL_REG3);
	val = (val & 0x3b) | (state << 2);
	lps22hh_write_register(handle, LPS22HH_REG_CTRL_REG3, val);
}
