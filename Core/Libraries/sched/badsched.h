/*
 * ../Libraries/sched/badsched.h
 *
 *  Created on: Aug 24, 2022
 *      Author: yevge
 */

#ifndef INC_BADSCHED_H_
#define INC_BADSCHED_H_

#include "../Libraries/debug/debug.h"
#include "stm32f4xx_hal.h"

#define PLATFORM_TIMER_GET __HAL_TIM_GET_COUNTER // set this to whatever the platform's timer function is
typedef TIM_HandleTypeDef platform_timer_handle_t;

typedef struct
{
	int32_t (*task)(void*, uint32_t);
	void* data;
	platform_timer_handle_t* timer;
	uint32_t period;
	uint32_t last;
} bad_task_t;

#define RUN_TASKS(routines) run_tasks(routines, sizeof(routines) / sizeof(bad_task_t))

void run_tasks(bad_task_t* taskv, uint16_t taskc);

#endif /* INC_BADSCHED_H_ */
