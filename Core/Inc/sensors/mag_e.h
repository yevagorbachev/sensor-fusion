/*
 * mag_e.h
 *
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_MAG_E_H_
#define INC_SENSORS_MAG_E_H_

#include "sensors/lis3mdl_lib.h"
#include "debug.h"
#include "stm32f4xx_hal.h"


void init_accel_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle);
int32_t accel_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t accel_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

#endif /* INC_SENSORS_MAG_E_H_ */