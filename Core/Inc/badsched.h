/*
 * badsched.h
 *
 *  Created on: Aug 24, 2022
 *      Author: yevge
 */

#ifndef INC_BADSCHED_H_
#define INC_BADSCHED_H_

#include "debug.h"
#include "stm32f4xx_hal.h"

typedef struct
{
	int32_t (*task)(void*, uint32_t);
	void* data;
	uint32_t period;
	uint32_t last;
} bad_task_t;

void run_tasks(TIM_HandleTypeDef* htim, bad_task_t* taskv, uint16_t taskc);

#endif /* INC_BADSCHED_H_ */
