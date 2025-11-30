#include "drone_can.hpp"
#include "canard_stm32_driver.h"
#include "dronecan_msgs.h"
#include "stm32l4xx_hal.h"
#include "../debug/debug.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>

static CanardInstance canard;
static constexpr size_t BUFFER_SIZE = 1024;
static uint8_t canard_buffer[BUFFER_SIZE];
static constexpr char const *NODE_NAME = "can-logger";
static uavcan_protocol_NodeStatus node_status;
static CAN_HandleTypeDef *hcan = nullptr;

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id);
// The actual ISR, modify this to your needs
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  // Receiving
  CanardCANFrame rx_frame;

  const uint64_t timestamp = HAL_GetTick() * 1000ULL;
  const int16_t rx_res = canardSTM32Recieve(hcan, CAN_RX_FIFO0, &rx_frame);

  if (rx_res < 0) {
    printf("Receive error %d\n", rx_res);
  } else if (rx_res > 0) {
    // Success - process the frame
	printf("ISR frame received");
    canardHandleRxFrame(&canard, &rx_frame, timestamp);
  }
}
*/

int DroneCan::init(CAN_HandleTypeDef *hcan1, const uint8_t node_id) {
	// configure CAN filters to receive everything
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(hcan1, &sFilterConfig) != HAL_OK) return -1;
	if (HAL_CAN_Start(hcan1) != HAL_OK) return -1;
	canardInit(&canard, canard_buffer, BUFFER_SIZE, onTransferReceived,
             shouldAcceptTransfer, nullptr);
	canardSetLocalNodeID(&canard, node_id);
	// setup ISR for CAN receive
	// if (HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return -1;
	hcan = hcan1;
	return 0;
}

void handleGetNodeInfo(CanardRxTransfer *transfer) {
  uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
  uavcan_protocol_GetNodeInfoResponse pkt;

  memset(&pkt, 0, sizeof(pkt));

  node_status.uptime_sec = HAL_GetTick() / 1000U;
  pkt.status = node_status;

  // TODO: update
  pkt.software_version.major = 0;
  pkt.software_version.minor = 0;
  pkt.software_version.optional_field_flags = 0;
  pkt.software_version.vcs_commit = 0; // should put git hash in here

  // TODO: update
  pkt.hardware_version.major = 0;
  pkt.hardware_version.minor = 0;

  getUniqueID(pkt.hardware_version.unique_id);

  strncpy((char *)pkt.name.data, "SimpleNode", sizeof(pkt.name.data));
  pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

  uint16_t total_size =
      uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

  canardRequestOrRespond(
      &canard, transfer->source_node_id, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
      UAVCAN_PROTOCOL_GETNODEINFO_ID, &transfer->transfer_id,
      transfer->priority, CanardResponse, &buffer[0], total_size);
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
  switch (transfer->transfer_type) {
  case CanardTransferTypeRequest:
    if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID) {
      handleGetNodeInfo(transfer);
    }
    break;
  case CanardTransferTypeResponse:
    break;
  case CanardTransferTypeBroadcast:

    break;
  }
}

bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id) {
  if (transfer_type == CanardTransferTypeRequest) {
    // check if we want to handle a specific service request
    switch (data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
      *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
      return true;
    }
    }
  }
  // we don't want any other messages
  return false;
}

void broadcastNodeStatus() {
  uint8_t status_buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];

  node_status.uptime_sec = HAL_GetTick() / 1000U;
  node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  node_status.sub_mode = 0;
  node_status.vendor_specific_status_code = 0;

  uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, status_buffer);

  // we need a static variable for the transfer ID. This is
  // incremeneted on each transfer, allowing for detection of packet
  // loss
  static uint8_t transfer_id;

  canardBroadcast(&canard, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                  UAVCAN_PROTOCOL_NODESTATUS_ID, &transfer_id,
                  CANARD_TRANSFER_PRIORITY_LOW, status_buffer, len);
}

void processCanardTxQueue() {
  // Transmitting

  for (const CanardCANFrame *tx_frame;
       (tx_frame = canardPeekTxQueue(&canard)) != NULL;) {
    const int16_t tx_res = canardSTM32Transmit(hcan, tx_frame);

    if (tx_res < 0) {
      printf("Transmit error %d.\r\n", tx_res);
      canardPopTxQueue(&canard);
    } else if (tx_res > 0) {
      printf("Successfully transmitted message.\r\n");
      canardPopTxQueue(&canard);
    } else {
      printf("Timeout! Trying again later.\r\n");
      break;
    }
  }
}

void writeFrameToLog(const CanardCANFrame * const frame) {
	uint32_t timestamp_millis = HAL_GetTick();
	uint32_t timestamp_s = timestamp_millis / 1000.0f;
	uint64_t frame_data = 0;
	for (uint64_t i = 0; i < frame->data_len; i++) {
		frame_data |= static_cast<uint64_t>(frame->data[i]) << (8 * (frame->data_len - 1 - i));
	}
	printf("(%ld.%ld) can%d %lx#%llx\r\n", timestamp_s, timestamp_millis, frame->iface_id, frame->id, frame_data);

}

void receiveSingleFrame() {
  // Receiving
  CanardCANFrame rx_frame;

  const uint64_t timestamp = HAL_GetTick() * 1000ULL;
  const int16_t rx_res = canardSTM32Recieve(hcan, CAN_RX_FIFO0, &rx_frame);
  if (rx_res < 0) {
    printf("Receive error %d.\r\n", rx_res);
  } else if (rx_res > 0) {
	  writeFrameToLog(&rx_frame);
	  canardHandleRxFrame(&canard, &rx_frame, timestamp);
  } else {
	  printf("No frames available or receive error of hcan!\r\n");
  }
}

void DroneCan::update() {
  processCanardTxQueue();
  receiveSingleFrame();
}
