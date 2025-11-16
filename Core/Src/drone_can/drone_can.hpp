#pragma once
#include "canard.h"
#include "stm32l4xx_hal.h"


namespace DroneCan {
  void init(CAN_HandleTypeDef *hcan);
  void update();
}
