/*
 * globals.c
 *
 *  Created on: Oct 9, 2022
 *      Author: yevge
 */

#include "../Devices/datatypes.h"


// preface variables with volatile to prevent compiler optimization

// calibration information
volatile angular_rate_t angular_rate_bias = {.x = 0.0f, .y = 0.0f, .z = 0.0f};
volatile uint32_t angular_rate_bias_n = 0;
volatile mag_field_t mag_hard_bias = {.x = -0.0015f, .y = -0.04f, .z = -0.5996f};
volatile soft_iron_t mag_soft_bias;

// raw data
volatile angular_rate_t angular_rate_raw;
volatile accel_t acceleration;
volatile mag_field_t mag_field_raw;

// calibrated data
volatile angular_rate_t angular_rate;
volatile mag_field_t mag_field;

volatile mag_field_t _mag_ringbuf_values[50];
volatile mag_ringbuf_t mag_ringbuf = MAG_RINGBUF(_mag_ringbuf_values);
volatile uint32_t angular_rate_time; // time of last angular rate measurement
volatile uint32_t mag_field_time; // time of last mag field measurement

/* JOHN CONSTANTS HERE */

volatile motor_output_t motor_output;
volatile uint32_t motor_output_time;

/* JOHN CONSTANTS END */
