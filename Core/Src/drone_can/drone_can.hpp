#pragma once
#include "canard.h"
#include "stm32l4xx_hal.h"


namespace DroneCan {
	// return 0 for OK, negative for error.
  int init(CAN_HandleTypeDef *hcan, const uint8_t node_id);
  void update();
}
