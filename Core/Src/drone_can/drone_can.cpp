#include "drone_can.hpp"
#include "libcanard/drivers/stm32/canard_stm32.h"
#include "dronecan_msgs.h"
#include "stm32l4xx_hal.h"
#include "../debug/debug.hpp"
#include "timestamp.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <inttypes.h>

static CanardInstance canard;
static constexpr size_t BUFFER_SIZE = 1024;
static uint8_t canard_buffer[BUFFER_SIZE];
static constexpr char const *NODE_NAME = "can-logger";
static uavcan_protocol_NodeStatus node_status;

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id);


int16_t DroneCan::init(CAN_HandleTypeDef *hcan1, const uint8_t node_id) {
	CanardSTM32CANTimings timings;
	if (!canardSTM32ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), 500000, &timings)) {
		timings.bit_rate_prescaler = 1;
		timings.bit_segment_1 = 6;
		timings.bit_segment_2 = 1;
		timings.max_resynchronization_jump_width = 1;
	}

	if (int16_t res = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal)) {
		return res;
	}
	canardInit(&canard, canard_buffer, BUFFER_SIZE, onTransferReceived,
             shouldAcceptTransfer, nullptr);
	canardSetLocalNodeID(&canard, node_id);
	// setup ISR for CAN receive
	if (HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return -1;
	return 0;
}


void getUniqueID(uint8_t id[16]){
	uint32_t unique_id[4];
	// Make Unique ID out of the 96-bit STM32 UID
	memset(id, 0, 16);
	unique_id[0] = HAL_GetUIDw0();
	unique_id[1] = HAL_GetUIDw1();
	unique_id[2] = HAL_GetUIDw2();
	unique_id[3] = HAL_GetUIDw1(); // repeating UIDw1 for this, no specific reason I chose this..
	memcpy(id, unique_id, 16);
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

  static uint8_t transfer_id;
  canardBroadcast(&canard, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                  UAVCAN_PROTOCOL_NODESTATUS_ID, &transfer_id,
                  CANARD_TRANSFER_PRIORITY_LOW, status_buffer, len);
}

void processCanardTxQueue() {
  for (const CanardCANFrame *tx_frame;
       (tx_frame = canardPeekTxQueue(&canard)) != NULL;) {
    const int16_t tx_res = canardSTM32Transmit(tx_frame);

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
	// printf with uint64_t is broken on stm
	uint32_t frame_data_h = frame_data >> 32;
	uint32_t frame_data_l = frame_data;
	if (frame_data_h) {
		printf("(%lu.%lu) can%u %lx#%lx%08lx\r\n",
				timestamp_s, timestamp_millis, frame->iface_id, frame->id, frame_data_h, frame_data_l);
	} else {
		printf("(%lu.%lu) can%u %lx#%lx\r\n",
				timestamp_s, timestamp_millis, frame->iface_id, frame->id, frame_data_l);
	}
}

int16_t receiveSingleFrame() {
  CanardCANFrame rx_frame;
  const int16_t rx_res = canardSTM32Receive(&rx_frame);
  if (rx_res == 0) {
	  printf("No frame in buffer\r\n");
  } else if (rx_res == 1) {
	  writeFrameToLog(&rx_frame);
	  uint64_t timestamp = HAL_GetTick() * 1000ULL;
	  canardHandleRxFrame(&canard, &rx_frame, timestamp);
  } else {
	  printf("Receive error %d.\r\n", rx_res);
  }
  return rx_res;
}

// ISR for CAN frame receive
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	receiveSingleFrame();
}

void DroneCan::update() {
  processCanardTxQueue();
  static Timestamp last_status = Timestamp::now();
  if (Timestamp::now() - last_status >= Duration::from_s(1)) {
	  last_status = last_status + Duration::from_s(1);
	  broadcastNodeStatus();
  }
}
