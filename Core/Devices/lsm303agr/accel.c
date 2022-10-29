#include "../Devices/lsm303agr/accel.h"

void init_accel_ctx(stmdev_ctx_t* ctx, I2C_HandleTypeDef* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) accel_read;
	ctx->write_reg = (stmdev_write_ptr) accel_write;
}

int32_t accel_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	return (int32_t)
		HAL_I2C_Mem_Read(handle, LSM303AGR_I2C_ADD_XL, reg | 0x80, 1, buf, len, LSM303AGR_XL_TIMEOUT);
}

int32_t accel_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	return (int32_t)
		HAL_I2C_Mem_Write(handle, LSM303AGR_I2C_ADD_XL, reg | 0x80, 1, buf, len, LSM303AGR_XL_TIMEOUT);
}

int32_t get_accel(stmdev_ctx_t* ctx, accel_t* output)
{
	int32_t ret;
	int16_t buf[3];
	lsm303agr_status_reg_a_t status;

	ret = lsm303agr_xl_status_get(ctx, &status);
	if (ret)
	{
		return ret;
	}
	if (!status.zyxda)
	{
		return -1;
	}

	ret = lsm303agr_acceleration_raw_get(ctx, buf);
	if (ret)
	{
		return ret;
	}

	float_t factor = 9.807f / 1000.0f; // miligee to m/s^2
	output->x = lsm303agr_from_fs_2g_hr_to_mg(buf[0]) * factor;
	output->y = lsm303agr_from_fs_2g_hr_to_mg(buf[1]) * factor;
	output->z = lsm303agr_from_fs_2g_hr_to_mg(buf[2]) * factor;
	return 0;
}
