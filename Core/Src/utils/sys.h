/*
 * sys.h
 *
 *  Created on: Dec 1, 2025
 *      Author: jakob
 */

#ifndef SRC_UTILS_SYS_H_
#define SRC_UTILS_SYS_H_
#include <stm32l4xx_hal.h>

// returns 0 on success, -1 on error
inline int usleep(unsigned long u_sec) {
	HAL_Delay(u_sec / 1000);
	return 0;
}





#endif /* SRC_UTILS_SYS_H_ */
