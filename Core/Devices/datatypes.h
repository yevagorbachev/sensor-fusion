/*
 * datatypes.h
 *
 *  Created on: Aug 24, 2022
 *      Author: yevge
 */

#ifndef INC_SENSORS_DATATYPES_H_
#define INC_SENSORS_DATATYPES_H_

#include <math.h> // for float_t
#include <stdint.h>

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

typedef struct {
	mag_field_t* values;
	int32_t size;
	int32_t position;
} mag_ringbuf_t;

#define MAG_RINGBUF(vals) {.values = vals, .size = sizeof(vals) / sizeof(mag_field_t), .position = 0}

typedef float_t motor_output_t;

#endif /* INC_SENSORS_DATATYPES_H_ */
