/*
 * gyro.c
 *
 *  Created on: Aug 23, 2022
 *      Author: yevge
 */

#include "sensors/gyro.h"

void init_gyro_ctx(stmdev_ctx_t* ctx, SPI_HandleTypeDef* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) gyro_read;
	ctx->write_reg = (stmdev_write_ptr) gyro_write;
}

int32_t gyro_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status;
	reg |= 0x40; // autoinc bit
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // enable CS (active-low)
	status = HAL_SPI_Transmit(handle, &reg, 1, I3G4250D_TIMEOUT);
	status |= HAL_SPI_Transmit(handle, buf, len, I3G4250D_TIMEOUT);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // disable CS
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#I3G4250D: Failed writing %d bytes to %.2hX\nData: ", len, reg ^ 0x3FU);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t gyro_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status;

	reg |= 0x80U | 0x40U; // read bit | autoinc bit
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // enable CS (active-low)
	status = HAL_SPI_Transmit(handle, &reg, 1, I3G4250D_TIMEOUT);
	status |= HAL_SPI_Receive(handle, buf, len, I3G4250D_TIMEOUT);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // disable CS
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#I3G4250D: Failed reading %d bytes from %.2hX", len, reg ^ 0x3FU);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

/** Checks status reg, puts angular rate to output (does conversion) if there is data on all axes
 * Returns 0 for success, -1 for no new data, 1-3 for HAL status
 */
int32_t get_angular_rate(stmdev_ctx_t* ctx, angular_rate_t* output)
{
	int16_t buf[3];
	int32_t ret;
	i3g4250d_status_reg_t status;

	ret = i3g4250d_status_reg_get(ctx, &status);
	if (ret)
	{
		return ret;
	}
	else if (status.zyxda)
	{
		ret = i3g4250d_angular_rate_raw_get(ctx, buf);
		if (ret)
		{
			return ret;
		}
		else
		{
			// UPDATE FACTOR AND COMMENT IF FULL-SCALE CONFIG CHANGES
			float_t factor = 70.0f / 1000.0f; // FS 2000 DPS -> DPS
			output->x = buf[0] * factor;
			output->y = buf[1] * factor;
			output->z = buf[2] * factor;
			return 0;
		}
	}
	else
	{
		return -1;
	}
}
