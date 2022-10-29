#include "mag.h"

/* Usage example:
 * stmdev_ctx_t mag_ctx;
 * double* raw_field;
 * I2C_HandleTypeDef hi2c;
 *
 * init_ctx(&mag_ctx, &hi2c);
 * init_mag(&mag_ctx);
 *
 * while (1) {get_mag(&raw_field);}
 */

/* Initalize magnetometer control structure
 *
 * ctx		pointer to control structure
 * handle	platform bus handle (can be used by mag_read and mag_write)
 */
void init_mag_ctx(stmdev_ctx_t* ctx, void* handle)
{
	ctx->handle = handle;
	ctx->read_reg = (stmdev_read_ptr) mag_read;
	ctx->write_reg = (stmdev_write_ptr) mag_write;
}

/* Setup: FS, PM, ODR, OM
 *
 * ctx		pointer to control structure
 */
void init_mag(stmdev_ctx_t* ctx)
{
#ifdef INC_DEBUG_H_
	printf("Initalizing external magnetometer\n");
#endif
	// initalize registers to default values from datasheet;
	uint8_t mag_e_controls[] = {0x10U, 0, 0x03U, 0, 0};
	lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG1,
			mag_e_controls, sizeof(mag_e_controls));

	// set desired output properties
	// make sure the conversions in get_mag are consistent with this full-scale
	lis3mdl_full_scale_set(ctx, LIS3MDL_4_GAUSS);
	lis3mdl_data_rate_set(ctx, LIS3MDL_UHP_155Hz);
	lis3mdl_operating_mode_set(ctx, LIS3MDL_CONTINUOUS_MODE);

	// read all of the registers, you can print them if you want
	lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1,
			mag_e_controls, sizeof(mag_e_controls));

#ifdef INC_DEBUG_H_
	printf("External magnetometer control registers: 0x");
	print_hex(mag_e_controls, sizeof(mag_e_controls));
#endif
}

/* Platform implementation to read from the LIS3MDL
 *
 * handle	platform bus handle (may be unused)
 * reg		starting register
 * buf		data destination
 * len		amount of data
 *
 * returns	status (ok, busy, error, etc.)
 * MANDATORY: Return value of 0 means success
 * MANDATORY: Do not return -1
 */
int32_t mag_read(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	/* PLATFORM READ IMPLEMENTATION */
	return (int32_t) HAL_I2C_Mem_Read((I2C_HandleTypeDef*) handle,
			LIS3MDL_I2C_ADD_L,
			reg | 0x80, 1, buf, len,
			LIS3MDL_TIMEOUT);
	/* PLATFORM READ IMPLEMENTATION END*/
}

/* Platform implementation to write to the LIS3MDL
 *
 * handle 	platform bus handle (may be unused)
 * reg		starting register
 * buf		data to be written
 * len 		amount of data to be written
 *
 * returns	status (ok, busy, error, etc.)
 * MANDATORY: Return value of 0 means success
 * MANDATORY: Do not return -1
 */
int32_t mag_write(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	/* PLATFORM WRITE IMPLEMENTATION */
	return (int32_t) HAL_I2C_Mem_Write((I2C_HandleTypeDef*) handle,
			LIS3MDL_I2C_ADD_L,
			reg | 0x80, 1, buf, len,
			LIS3MDL_TIMEOUT);
	/* PLATFORM WRITE IMPLEMENTATION END*/
}

/* Use the control structure's read/write functions to get status, then read magnetic field
 * MANDATORY: Check the conversion function and factor to make sure the output units are
 * what you expect
 *
 * ctx		device control structure
 * output	pointer to where the result should be written
 *
 * returns	status: -1 no data, 0 ok, otherwise - interface status as returned by mag_read
 */
int32_t get_mag(stmdev_ctx_t* ctx, mag_field_t* output)
{
	int32_t ret; // store interface return value
	int16_t buf[3]; // store raw magnetic data
	lis3mdl_status_reg_t status; // store status register contents

	ret = lis3mdl_status_get(ctx, &status);
	if (ret)
	{
		return ret; // failed to read status register
	}
	if (!status.zyxda)
	{
		return -1; // no data ready
	}
	ret = lis3mdl_magnetic_raw_get(ctx, buf);
	if (ret)
	{
		return ret; // failed to read raw data
	}

	// nothing failed and we have new data -> convert and store it
	float_t factor = 1;

	output->x = lis3mdl_from_fs4_to_gauss(buf[0]) * factor;
	output->y = lis3mdl_from_fs4_to_gauss(buf[1]) * factor;
	output->z = lis3mdl_from_fs4_to_gauss(buf[2]) * factor;

	return 0;
}

/* Add a measurement to the ringbuffer containing the magnetometer measurements
 * if and only if the read succeeded
 *
 * measurements		pointer to ringbuffer containing magnetic field measurements
 * value			new value
 */

void put_mag(mag_ringbuf_t* measurements, mag_field_t value)
{
	measurements->values[measurements->position] = value;
	measurements->position = (measurements->position + 1) % measurements->size;
}

/* Take the average of the ringbuffer's contents
 *
 * measurements		pointer to ringbuffer containing magnetic field measurements
 *
 * returns			the average of the measurements in the ringbuffer
 */
mag_field_t get_smooth_mag(mag_ringbuf_t* measurements)
{
	mag_field_t mean = {.x = 0, .y = 0, .z = 0};
	for (int i = 0; i < measurements->size; i++)
	{
		mean.x += measurements->values[i].x;
		mean.y += measurements->values[i].y;
		mean.z += measurements->values[i].z;
	}
	mean.x /= measurements->size;
	mean.y /= measurements->size;
	mean.z /= measurements->size;

	return mean;
}

/* Get heading using principal arctangent
 * MANDATORY: Magnetometer must have z axis pointing down
 *
 * value	mag value to convert
 *
 * returns	heading 0-359, assuming positive x is the "front"
 */
float_t get_heading(mag_field_t value)
{
	float_t angle = (float_t) atan2(value.y, value.x) * 180.0f / 3.141592654f;
	return (angle > 0) ? angle : 360 + angle;
}
