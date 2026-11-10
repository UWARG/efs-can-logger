#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Confirmed on-disk CAN record format: 20 bytes per frame. */
typedef struct {
  uint32_t timestamp_ms;
  uint32_t arbitration_id;
  uint8_t  frame_flags;
  uint8_t  data_length_code;
  uint8_t  data_bytes[8];
  uint8_t  padding[2];
} CanLogRecord;

/*
 * Initialize the CAN receiver after MX_CAN1_Init() has configured the
 * peripheral. This configures the receive filter, starts CAN1, and enables
 * the RX FIFO0 notification used by the receiver callback.
 */
bool CanReceiver_Init(void);
