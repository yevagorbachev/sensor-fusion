/*
 * debug.c
 *
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */
#include "debug.h"

// print bytes as hex array

void print_hex(const uint8_t* buf, int len)
{
	for (int i = 0; i < len; i++)
	{
		printf("%.2hX", buf[i]);
	}
	printf("\n");
}

// print HAL status

void print_hal(HAL_StatusTypeDef status)
{
	switch (status)
	{
		case HAL_OK:
			printf("HAL OK\n");
			break;
		case HAL_ERROR:
			printf("HAL ERROR\n");
			break;
		case HAL_TIMEOUT:
			printf("HAL TIMEOUT\n");
			break;
		case HAL_BUSY:
			printf("HAL BUSY\n");
			break;
	}
}
