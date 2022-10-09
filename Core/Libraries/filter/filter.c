/*
 * filter.c
 *
 *  Created on: Oct 9, 2022
 *      Author: yevge
 */

#include "filter.h"

mag_field_t apply_mag_cal(mag_field_t raw, mag_field_t hard_offset, soft_iron_t soft_offset)
{
	mag_field_t calibrated = {
		.x = raw.x - hard_offset.x,
		.y = raw.y - hard_offset.y,
		.z = raw.z - hard_offset.z,
	};
	return calibrated;
}

angular_rate_t apply_gyro_cal(angular_rate_t raw, angular_rate_t bias)
{
	angular_rate_t calibrated = {
		.x = raw.x - bias.x,
		.y = raw.y - bias.y,
		.z = raw.z - bias.z,
	};
	return calibrated;
}

