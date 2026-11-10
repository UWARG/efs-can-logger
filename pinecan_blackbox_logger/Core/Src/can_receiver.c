#include "can_receiver.h"

#include "blackbox_logger.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

extern CAN_HandleTypeDef hcan1;

enum
{
  CAN_RECORD_FLAG_EXTENDED = 1U << 0,
  CAN_RECORD_FLAG_REMOTE = 1U << 1
};

_Static_assert(sizeof(CanLogRecord) == 20U,
               "CanLogRecord layout must remain 20 bytes");

static bool receiver_initialized = false;

bool CanReceiver_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  if (receiver_initialized) {
    return true;
  }

  /* Accept all frames initially. Narrow this once the required CAN IDs are known. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
    return false;
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    return false;
  }

  if (HAL_CAN_ActivateNotification(
        &hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL) != HAL_OK) {
    return false;
  }

  receiver_initialized = true;
  return true;
}

/*
 * Called by the HAL after CAN1_RX0_IRQHandler() dispatches a FIFO0 event.
 * This function must remain short: no filesystem access, blocking calls, or
 * dynamic allocation belong in the interrupt context.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  CanLogRecord record = {0};
  uint8_t payload_size;

  if (hcan != &hcan1) {
    return;
  }

  if (HAL_CAN_GetRxMessage(
        hcan,
        CAN_RX_FIFO0,
        &header,
        record.data_bytes) != HAL_OK) {
    return;
  }

  record.timestamp_ms = HAL_GetTick();

  if (header.IDE == CAN_ID_EXT) {
    record.arbitration_id = header.ExtId;
    record.frame_flags |= CAN_RECORD_FLAG_EXTENDED;
  } else {
    record.arbitration_id = header.StdId;
  }

  if (header.RTR == CAN_RTR_REMOTE) {
    record.frame_flags |= CAN_RECORD_FLAG_REMOTE;
  }

  payload_size = (header.DLC <= sizeof(record.data_bytes))
               ? (uint8_t)header.DLC
               : (uint8_t)sizeof(record.data_bytes);
  record.data_length_code = payload_size;

  /* The logger copies the record into its active ISR-safe buffer. */
  BlackboxLogger_PushFromISR(&record, sizeof(record));
}
