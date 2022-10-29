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

#include<math.h>

#include "lis3mdl_reg.h"
#include "../datatypes.h"
#include "stm32f4xx_hal.h"

#define LIS3MDL_TIMEOUT 10

// Initalize control structure
void init_mag_ctx(stmdev_ctx_t* ctx, void* handle);

// Reset device to defaults and configure
void init_mag(stmdev_ctx_t* mag_ctx_ptr);

// Platform implementation to read <len> bytes to <buf> from the device at <reg>
int32_t mag_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

// Platform implementation to write <len> bytes from <buf> to the device at <reg>
int32_t mag_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

// Check status and get output if there is new data
int32_t get_mag(stmdev_ctx_t* ctx, mag_field_t* output);

// add measurement to the ringbuffer
void put_mag(mag_ringbuf_t* measurements, mag_field_t value);

// average of the contents of the ringbuffer
mag_field_t get_smooth_mag(mag_ringbuf_t* measurements);

// x,y to heading
float_t get_heading(mag_field_t value);

#endif /* INC_SENSORS_MAG_E_H_ */
