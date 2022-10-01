/*
 * ../Libraries/debug/debug.h
 *
 *  Created on: Aug 22, 2022
 *      Author: yevge
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include<stdio.h>
#include<string.h>
#include<stddef.h>
#include<stm32f4xx_hal.h>


void print_hex(const uint8_t* buf, int len);
void print_hal(HAL_StatusTypeDef status);

#endif /* INC_DEBUG_H_ */
