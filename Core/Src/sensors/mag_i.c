#include "sensors/mag_i.h"

void init_mag_i_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) mag_i_read;
	ctx->write_reg = (stmdev_write_ptr) mag_i_write;
}

int32_t mag_i_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, LSM303AGR_I2C_ADD_MG, reg | 0x80, 1, buf, len, LSM303AGR_MG_TIMEOUT);
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#LSM303AGR: Failed reading %d bytes from %.2hX: ", len, reg);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t mag_i_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, LSM303AGR_I2C_ADD_MG, reg | 0x80, 1, buf, len, LSM303AGR_MG_TIMEOUT);
#ifdef INC_DEBUG_H_
	if (status != HAL_OK)
	{
		printf("#RED#LSM303AGR: Failed writing %d bytes to %.2hX: ", len, reg);
		print_hal(status);
	}
#endif
	return (int32_t) status;
}

int32_t get_mag_i(stmdev_ctx_t* ctx, mag_field_t* output)
{
	int32_t ret;
	int16_t buf[3];
	lsm303agr_status_reg_m_t status;

	ret = lsm303agr_mag_status_get(ctx, &status);
	if (ret)
	{
		return ret;
	}
	else if (status.zyxda)
	{
		ret = lsm303agr_magnetic_raw_get(ctx, buf);
		if (ret)
		{
			return ret;
		}
		else
		{
			float_t factor = 1.0f / 1000.0f; // mgauss -> gauss
			output->x = lsm303agr_from_lsb_to_mgauss(buf[0]) * factor;
			output->y = lsm303agr_from_lsb_to_mgauss(buf[1]) * factor;
			output->z = lsm303agr_from_lsb_to_mgauss(buf[1]) * factor;
		}
		return 0;
	}
	else
	{
		return -1;
	}
}
