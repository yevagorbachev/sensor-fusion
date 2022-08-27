/*
 * mag_e.h
 * Implements ST's API for LIS3MDL magnetometer
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

// Implementation note: The breakout connects via a Stemma QT connector with the pinout
// RED -> GRAY: 3V3
// BLACK -> BLACK: GND
// BLUE -> VIOLET: SDA - PB9 for I2C1
// YELLOW -> WHITE: SCL - PB6 for I2C1

#ifndef INC_SENSORS_MAG_E_H_
#define INC_SENSORS_MAG_E_H_

#include "sensors/lis3mdl_reg.h"
#include "sensors/datatypes.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

#define LIS3MDL_TIMEOUT 10
#define MAG_E_SCALE LIS3MDL_4_GAUSS

void init_mag_e_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle);
int32_t mag_e_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t mag_e_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

int32_t get_mag_e(stmdev_ctx_t* ctx, mag_field_t* output);
#endif /* INC_SENSORS_MAG_E_H_ */
