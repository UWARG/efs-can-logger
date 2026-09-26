// blackbox_logger.h
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool BlackboxLogger_Init(const char *filename);

/* Safe to call from the CAN receive callback. Rejects and counts records after a storage fault. */
bool BlackboxLogger_PushFromISR(const void *record, size_t size);

/* Call repeatedly from the main loop. May block in f_write().
 * A failed or short write latches a fault; subsequent calls do not retry. */
void BlackboxLogger_Service(void);

/* Flush a partially filled buffer and synchronize the filesystem. */
bool BlackboxLogger_Flush(void);

uint32_t BlackboxLogger_GetDroppedRecordCount(void);

/* Latched write fault. Pending buffers are retained, but logging is stopped.
 * No automatic recovery: do not reinitialize a running logger to clear this. */
bool BlackboxLogger_HasStorageError(void);
