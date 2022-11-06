/*
 * filter.h
 *
 *  Created on: Oct 5, 2022
 *      Author: yevge
 */

#ifndef LIBRARIES_FILTER_FILTER_H_
#define LIBRARIES_FILTER_FILTER_H_


#include "../../Devices/datatypes.h"

mag_field_t apply_mag_cal(mag_field_t raw, mag_field_t hard_offset);
angular_rate_t apply_gyro_cal(angular_rate_t raw, angular_rate_t bias);


#endif /* LIBRARIES_FILTER_FILTER_H_ */
