/*
 * gyro.c
 *
 *  Created on: Aug 23, 2022
 *      Author: yevge
 */

#include "gyro.h"

void init_gyro_ctx(stmdev_ctx_t* ctx, void* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) gyro_read;
	ctx->write_reg = (stmdev_write_ptr) gyro_write;
}

// Writes to registers using SPI
int32_t gyro_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status;
	reg |= 0x40U; // autoinc bit
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // enable CS (active-low)
	status = HAL_SPI_Transmit(handle, &reg, 1, I3G4250D_TIMEOUT);
	status |= HAL_SPI_Transmit(handle, buf, len, I3G4250D_TIMEOUT);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // disable CS
	return (int32_t) status;
}

// Reads registers using SPI
int32_t gyro_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status;

	reg |= 0x80U | 0x40U; // read bit | autoinc bit
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // enable CS (active-low)
	status = HAL_SPI_Transmit(handle, &reg, 1, I3G4250D_TIMEOUT);
	status |= HAL_SPI_Receive(handle, buf, len, I3G4250D_TIMEOUT);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // disable CS
	return (int32_t) status;
}

// converts angular rate from digits to degrees per second
void convert_angular_rate(int16_t* buf, angular_rate_t* output)
{
	float_t factor = I3G4250D_FACTOR / 1000.0f; // LSB -> mDPS -> DPS
	output->x = buf[0] * factor;
	output->y = buf[1] * factor;
	output->z = buf[2] * factor;
}

/** Checks status reg, puts angular rate to output (does conversion) if there is data on all axes
 * Returns 0 for success, -1 for no new data, 1-3 for HAL_StatusTypeDef
 */
int32_t get_angular_rate(stmdev_ctx_t* ctx, angular_rate_t* output)
{
	int16_t buf[3];
	int32_t ret;
	i3g4250d_status_reg_t status;

	ret = i3g4250d_status_reg_get(ctx, &status);

	if (ret)
	{
		return ret; // failed to read status register
	}
	if (!status.zyxda)
	{
		return -1; // no data ready
	}
	ret = i3g4250d_angular_rate_raw_get(ctx, buf);
	if (ret)
	{
		return ret;
	}


	convert_angular_rate(buf, output);
	return 0;
}

/** Puts contents of angular rate registers to output - does not check zyxda
 * We're using this in the DRDY ISR
 * Returns 0 for success, 1-3 for HAL_StatusTypeDef
 */
int32_t get_angular_rate_nocheck(stmdev_ctx_t* ctx, angular_rate_t* output)
{
	int16_t buf[3];
	int32_t ret;
	ret = i3g4250d_angular_rate_raw_get(ctx, buf);
	if (ret)
	{
		return ret;
	}

	convert_angular_rate(buf, output);
	return 0;
}
