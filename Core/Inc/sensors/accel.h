/*
 * accel.h
 * Library for LSM303AGR accelerometer
 * Condenses driver down to our use case
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_ACCEL_H_
#define INC_SENSORS_ACCEL_H_

#include "sensors/lsm303agr_lib.h"
#include "main.h"

#define ACCEL_UNIT_MPS // convert miligee to meters per second

#define ACCEL_SCALE LSM303AGR_2g
#define ACCEL_MODE LSM303AGR_HR_12bit
#define LSM303AGR_A_TIMEOUT 100

void init_accel_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle);
int32_t accel_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t accel_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

typedef struct {
	float_t x;
	float_t y;
	float_t z;
} accel_t;

int32_t get_accel(stmdev_ctx_t* ctx, accel_t* output);
#endif /* INC_SENSORS_ACCEL_H_ */
