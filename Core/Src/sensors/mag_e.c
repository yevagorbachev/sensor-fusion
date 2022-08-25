#include "sensors/mag_e.h"

void init_mag_e_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) mag_e_read;
	ctx->write_reg = (stmdev_write_ptr) mag_e_write;
}

int32_t mag_e_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_L, reg | 0x80, 1, buf, len, LIS3MDL_TIMEOUT);
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#LSM303AGR: Failed reading %d bytes from %.2hX\n", len, reg);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t mag_e_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_L, reg | 0x80, 1, buf, len, LIS3MDL_TIMEOUT);
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#LSM303AGR: Failed writing %d bytes to %.2hX\n", len, reg);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t get_mag_e(stmdev_ctx_t* ctx, mag_field_t* output)
{
	int32_t ret;
	int16_t buf[3];
	lis3mdl_status_reg_t status;

	ret = lis3mdl_status_get(ctx, &status);
	if (ret)
	{
		return ret;
	}
	else if (status.zyxda)
	{
		ret = lis3mdl_magnetic_raw_get(ctx, buf);
		if (ret)
		{
			return ret;
		}
		else
		{
			float_t factor = 1;
			output->x = lis3mdl_from_fs4_to_gauss(buf[0]) * factor;
			output->y = lis3mdl_from_fs4_to_gauss(buf[1]) * factor;
			output->z = lis3mdl_from_fs4_to_gauss(buf[2]) * factor;
		}
		return 0;
	}
	else
	{
		return -1;
	}
}
