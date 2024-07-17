#ifndef INC_LPS22HH_H_
#define INC_LPS22HH_H_

#include "inttypes.h"

#define LPS22HH_HANDLE             hspi1
#define LPS22HH_COUNT 8
#define LPS22HH_BUF_SIZE 5*LPS22HH_COUNT

#define LPS22HH_ADDRESS            0xBA
#define LPS22HH_DEVICE_NAME        0b10110011

#define LPS22HH_REG_IF_CTRL        0x0E
#define LPS22HH_REG_WHO_AM_I       0x0F
#define LPS22HH_REG_CTRL_REG1      0x10
#define LPS22HH_REG_CTRL_REG2      0x11
#define LPS22HH_REG_CTRL_REG3      0x12
#define LPS22HH_REG_STATUS         0x27
#define LPS22HH_REG_PRESS_OUT_XL   0x28
#define LPS22HH_REG_PRESS_OUT_L    0x29
#define LPS22HH_REG_PRESS_OUT_H    0x2A
#define LPS22HH_REG_TEMP_OUT_L     0x2B
#define LPS22HH_REG_TEMP_OUT_H     0x2C

enum Sensor {
	Lps1,
	Lps2,
	Lps3,
	Lps4,
	Lps5,
	Lps6,
	Lps7,
	Lps8,
};

void Lps22hh_Init(void);
void Lps22hh_Update(void);
void Lps22hh_Data(uint8_t *buf);

#endif
