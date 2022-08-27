/*
 * gyro.h
 * Implements ST's API for I3G4250D magnetometer
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_GYRO_H_
#define INC_SENSORS_GYRO_H_

#include <sensors/i3g4250d_reg.h>
#include "sensors/datatypes.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

#define GYRO_SCALE I3G4250D_500dps
#define I3G4250D_TIMEOUT 10

void init_gyro_ctx(stmdev_ctx_t* ctx, SPI_HandleTypeDef* handle);
int32_t gyro_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t gyro_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

void convert_angular_rate(int16_t* buf, angular_rate_t* output);
int32_t get_angular_rate(stmdev_ctx_t* ctx, angular_rate_t* output);
int32_t get_angular_rate_nocheck(stmdev_ctx_t* ctx, angular_rate_t* output);
#endif /* INC_SENSORS_GYRO_H_ */
