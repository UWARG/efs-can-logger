/*
 * debug.cpp
 *
 *  Created on: Nov 30, 2025
 *      Author: jakob
 */
#include "debug.hpp"
#include <stdio.h>
#include "stm32l4xx_hal.h"

int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
}




