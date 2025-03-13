#ifndef INC_LPS22HH_H_
#define INC_LPS22HH_H_

#include "stdint.h"
#include "main.h"

#define LPS22HH_DATA_SIZE 5
#define LPS22HH_BUF_SIZE LPS22HH_COUNT*LPS22HH_DATA_SIZE

#define LPS22HH_ADDRESS 0xBA
#define LPS22HH_ID 0xB3 //0b10110011

//registers
#define LPS22HH_REG_INTERRUPT_CFG 0x0B //r-w

#define LPS22HH_REG_THS_P_L 0x0C //r-w
#define LPS22HH_REG_THS_P_H 0x0D //r-w

#define LPS22HH_REG_IF_CTRL 0x0E //r-w

#define LPS22HH_REG_WHO_AM_I 0x0F //r

#define LPS22HH_REG_CTRL_REG1 0x10 //r-w
#define LPS22HH_REG_CTRL_REG2 0x11 //r-w
#define LPS22HH_REG_CTRL_REG3 0x12 //r-w

#define LPS22HH_REG_FIFO_CTRL 0x13 //r-w
#define LPS22HH_REG_FIFO_WTM  0x14 //r-w

#define LPS22HH_REG_REF_P_L 0x15 //r
#define LPS22HH_REG_REF_P_H 0x16 //r

#define LPS22HH_REG_RPDS_L 0x18 //r-w
#define LPS22HH_REG_RPDS_H 0x19 //r-w

#define LPS22HH_REG_INT_SOURCE 0x24 //r

#define LPS22HH_REG_FIFO_STATUS1 0x25 //r
#define LPS22HH_REG_FIFO_STATUS2 0x26 //r

#define LPS22HH_REG_STATUS 0x27 //r

#define LPS22HH_REG_PRESS_XL 0x28 //r
#define LPS22HH_REG_PRESS_L  0x29 //r
#define LPS22HH_REG_PRESS_H  0x2A //r

#define LPS22HH_REG_TEMP_L  0x2B //r
#define LPS22HH_REG_TEMP_H  0x2C //r

#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_XL 0x78 //r
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_L 0x79 //r
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_H 0x7A //r

#define LPS22HH_REG_FIFO_DATA_OUT_TEMP_L 0x7B //r
#define LPS22HH_REG_FIFO_DATA_OUT_TEMP_H 0x7C //r

enum Lps22hh_Odr {
	Lps22hh_OdrOnce = 0u,
	Lps22hh_Odr1 = 1u,
	Lps22hh_Odr10 = 2u,
	Lps22hh_Odr25 = 3u,
	Lps22hh_Odr50 = 4u,
	Lps22hh_Odr75 = 5u,
	Lps22hh_Odr100 = 6u,
	Lps22hh_Odr200 = 7u,
};

enum Lps22hh_Filt {
	Lps22hh_Filt2 = 0u,
	Lps22hh_Filt9 = 2u,
	Lps22hh_Filt20 = 3u,
};

struct Lps22hh_Handle {
	// configuration
    SPI_HandleTypeDef* hspi;//10Mhz max

    GPIO_TypeDef* csPort;
    uint16_t csPin;


    uint16_t intPin;
    // internal
    uint8_t init;

    uint8_t* data;
};

void Lps22hh_Init(struct Lps22hh_Handle* handle);

uint8_t Lps22hh_ExtFlag(struct Lps22hh_Handle* handle, uint16_t pin);
void Lps22hh_ExtHandler(struct Lps22hh_Handle* handle);

void Lps22hh_Reset(struct Lps22hh_Handle* handle);

void Lps22hh_SetODR(struct Lps22hh_Handle* handle, enum Lps22hh_Odr odr);
void Lps22hh_SetFilter(struct Lps22hh_Handle* handle, enum Lps22hh_Filt filter);
void Lps22hh_SetInterrupt(struct Lps22hh_Handle* handle, uint8_t state);

#endif
