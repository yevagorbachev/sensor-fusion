/*
 * ldm303dlhc.h
 *
 *  Created on: Aug 17, 2022
*      Author: Yevgeniy Gorbachev
 */

#ifndef INC_LSM303DLHC_H_
#define INC_LSM303DLHC_H_

#define LSM303_A_READ_ADDRESS 0x33
#define LSM303_A_WRITE_ADDRESS 0x32

#define LSM303_A_CTL(index) 0x19 + (index)
#define LSM303_A_REF 0x26
#define LSM303_A_STATUS 0x27
#define LSM303_A_DATA 0x28

HAL_StatusTypeDef lsm303_read(I2C_HandleTypeDef* handle, uint8_t reg,  uint8_t* buf, uint16_t len);
HAL_StatusTypeDef lsm303_write(void* handle, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* INC_LSM303DLHC_H_ */
