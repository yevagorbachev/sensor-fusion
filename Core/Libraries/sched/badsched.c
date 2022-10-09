/*
 * badsched.c
 * Bad scheduler
 * Runs the set tasks every specified period
 * Not preemptive
 *  Created on: Aug 24, 2022
 *      Author: yevge
 */

#include "badsched.h"

void run_tasks(bad_task_t* taskv, uint16_t taskc)
{
	uint32_t time;
//	int32_t code;
	for (uint16_t i = 0; i < taskc; i++)
	{
		time = PLATFORM_TIMER_GET(taskv[i].timer);
		if (time - taskv[i].last > taskv[i].period)
		{
			taskv[i].task(taskv[i].data, time - taskv[i].last);
			taskv[i].last = time;
		}
	}
}
