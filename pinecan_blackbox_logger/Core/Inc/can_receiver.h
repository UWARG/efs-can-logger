#pragma once

#include <stdbool.h>
#include <stdint.h>

// TEMP for development
typedef struct {
  uint32_t timestamp_ms;
  uint32_t id;
  uint8_t  flags;
  uint8_t  dlc;
  uint8_t  data[8];
  uint8_t  reserved[2];
} CanLogRecord;

/*
 * Initialize the CAN receiver after MX_CAN1_Init() has configured the
 * peripheral. This configures the receive filter, starts CAN1, and enables
 * the RX FIFO0 notification used by the receiver callback.
 */
bool CanReceiver_Init(void);
