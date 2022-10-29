/*
 * state.h
 *
 *  Created on: Oct 9, 2022
 *      Author: yevge
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

// preface variables from state.c with extern

// calibration information
extern angular_rate_t angular_rate_bias;
extern uint32_t angular_rate_bias_n;
extern mag_field_t mag_hard_bias;
extern soft_iron_t mag_soft_bias;

// raw data
extern angular_rate_t angular_rate_raw;
extern accel_t acceleration;
extern mag_field_t mag_field_raw;

// calibrated data
extern angular_rate_t angular_rate;
extern mag_field_t mag_field;
extern mag_ringbuf_t mag_ringbuf;
extern uint32_t angular_rate_time; // time of last angular rate measurement
extern uint32_t mag_field_time; // time of last mag field measurement

/* JOHN CONSTANTS HERE */

/* JOHN CONSTANTS END */

#endif /* INC_STATE_H_ */
