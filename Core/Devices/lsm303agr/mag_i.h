/*
 * mag_i.h
 * Implements ST's API for LSM303AGR magnetometer
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_MAG_I_H_
#define INC_SENSORS_MAG_I_H_

#include "../datatypes.h"
#include "stm32f4xx_hal.h"
#include "lsm303agr_reg.h"

#define LSM303AGR_MG_TIMEOUT 10

void init_mag_i_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle);
int32_t mag_i_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t mag_i_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

int32_t get_mag_i(stmdev_ctx_t* ctx, mag_field_t* output);
#endif /* INC_SENSORS_MAG_I_H_ */
