/*
 * gyro.h
 *
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_GYRO_H_
#define INC_SENSORS_GYRO_H_

#include <malloc.h>
#include "sensors/i3g4250d_lib.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

#define GYRO_SCALE I3G4250D_2000dps
#define GYRO_RATE I3G4250D_ODR_100Hz
#define I3G4250D_TIMEOUT 100

void init_gyro_ctx(stmdev_ctx_t* ctx, SPI_HandleTypeDef* handle);
int32_t gyro_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t gyro_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

typedef struct {
	float_t x;
	float_t y;
	float_t z;
} angular_rate_t;

int32_t get_angular_rate(stmdev_ctx_t* ctx, angular_rate_t* output);
#endif /* INC_SENSORS_GYRO_H_ */
