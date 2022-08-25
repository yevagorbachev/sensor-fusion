/*
 * datatypes.h
 *
 *  Created on: Aug 24, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_DATATYPES_H_
#define INC_SENSORS_DATATYPES_H_

typedef struct {
	float_t x;
	float_t y;
	float_t z;
	char* unit;
} accel_t;


typedef struct {
	float_t x;
	float_t y;
	float_t z;
	char* unit;
} angular_rate_t;


typedef struct {
	float_t x;
	float_t y;
	float_t z;
	char* unit;
} mag_field_t;

#endif /* INC_SENSORS_DATATYPES_H_ */
