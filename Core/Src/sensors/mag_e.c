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
