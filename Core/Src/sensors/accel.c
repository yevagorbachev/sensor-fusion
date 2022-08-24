#include "sensors/accel.h"

void init_accel_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) accel_read;
	ctx->write_reg = (stmdev_write_ptr) accel_write;
}

int32_t accel_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, LSM303AGR_I2C_ADD_XL, reg | 0x80, 1, buf, len, LSM303AGR_XL_TIMEOUT);
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#LSM303AGR: Failed reading %d bytes from %.2hX\n", len, reg);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t accel_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, LSM303AGR_I2C_ADD_XL, reg | 0x80, 1, buf, len, LSM303AGR_XL_TIMEOUT);
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#LSM303AGR: Failed writing %d bytes to %.2hX\n", len, reg);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t get_accel(stmdev_ctx_t* ctx, accel_t* output)
{
	int16_t buf[3];
	int32_t ret = lsm303agr_acceleration_raw_get(ctx, buf);
	if ((HAL_StatusTypeDef) ret == HAL_OK)
	{
#ifdef ACCEL_UNIT_MPS
		float_t factor = 9.807f / 1000.0f;
#else
		float_t factor = 1;
#endif
		output->x = lsm303agr_from_fs_2g_hr_to_mg(buf[0]) * factor;
		output->y = lsm303agr_from_fs_2g_hr_to_mg(buf[1]) * factor;
		output->z = lsm303agr_from_fs_2g_hr_to_mg(buf[2]) * factor;
	}
	return ret;
}
