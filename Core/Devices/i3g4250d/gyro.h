/*
 * gyro.h
 * Implements ST's API for I3G4250D magnetometer
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_GYRO_H_
#define INC_SENSORS_GYRO_H_

#include "i3g4250d_reg.h"
#include "../datatypes.h"
#include "stm32f4xx_hal.h"

#define GYRO_FS_dps 500 // 245, 500, 2000
#define I3G4250D_ODR I3G4250D_ODR_400Hz
#define I3G4250D_TIMEOUT 10

#if GYRO_FS_dps == 245
#define I3G4250D_FS I3G4250D_245dps
#define I3G4250D_FACTOR 8.75f // can vary 7.4 to 10.1
#elif GYRO_FS_dps == 500
#define I3G4250D_FS I3G4250D_500dps
#define I3G4250D_FACTOR 17.5f // can vary 14.8 to 19.8
#elif GYRO_FS_dps == 2000
#define I3G4250D_FS I3G4250D_2000dps
#define I3G4250D_FACTOR 70f // can vary 59.2 to 79.3
#else
#error Invalid gyroscope full-scale
#endif

// factors in mdps/digit

void init_gyro_ctx(stmdev_ctx_t* ctx, void* handle);
int32_t gyro_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);
int32_t gyro_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len);

void convert_angular_rate(int16_t* buf, angular_rate_t* output);
int32_t get_angular_rate(stmdev_ctx_t* ctx, angular_rate_t* output);
int32_t get_angular_rate_nocheck(stmdev_ctx_t* ctx, angular_rate_t* output);
#endif /* INC_SENSORS_GYRO_H_ */
