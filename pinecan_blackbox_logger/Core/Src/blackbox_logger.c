// blackbox_logger.c
#include "blackbox_logger.h"
#include "fatfs.h"
#include "main.h"

#include <string.h>

#define LOG_BUFFER_SIZE 6144U

typedef struct {
    uint8_t data[LOG_BUFFER_SIZE];
    volatile size_t used;
} LogBuffer;

static LogBuffer buffers[2];

static volatile uint8_t active_index; //Receiving data
static volatile int8_t ready_index; //Ready to write/writing to SD card
static volatile bool writer_busy;
static volatile bool storage_error;
static volatile uint32_t dropped_records;


// For testing
static volatile uint32_t last_write_ms;
static volatile uint32_t max_write_ms;
static volatile size_t last_used;
static volatile uint32_t sync_time;
static volatile uint32_t max_sync_time;


static FATFS filesystem;
static FIL log_file;

bool BlackboxLogger_Init(const char *filename) {
    active_index = 0;
    ready_index = -1; //Signal no buffer is waiting to write
    writer_busy = false;
    storage_error = false;
    dropped_records = 0;

    last_write_ms = 0;
    max_write_ms = 0;
    last_used = 0;

    sync_time = 0;
    max_sync_time = 0;

    buffers[0].used = 0;
    buffers[1].used = 0;

    FRESULT mount_stat = f_mount(&filesystem, "", 1);

    if (mount_stat != FR_OK) {
        f_unlink(filename);
        f_mount(&filesystem, "", 1);
    }


    return f_open(&log_file, filename, FA_OPEN_APPEND | FA_WRITE) == FR_OK;
}

bool BlackboxLogger_PushFromISR(const void *record, size_t size) {
    if (record == NULL || size == 0 || size > LOG_BUFFER_SIZE) {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (storage_error) {
        dropped_records++;
        __set_PRIMASK(primask);
        return false;
    }

    LogBuffer *buffer = &buffers[active_index];

    if (buffer->used + size > LOG_BUFFER_SIZE) {
        if(ready_index >= 0) {
            dropped_records++;
            __set_PRIMASK(primask);
            return false;
        }

        ready_index = (int8_t)active_index;
        active_index ^= 1U;

        buffer = &buffers[active_index];
        buffer->used = 0;
    }

    memcpy(&buffer->data[buffer->used], record, size);
    buffer->used += size;

    if (primask == 0U) {
        __enable_irq();
    }

    return true;
}

void BlackboxLogger_Service(void) {
    int8_t claimed_index = -1;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!storage_error && (ready_index >= 0) && !writer_busy) {
        claimed_index = ready_index;
        writer_busy = true;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    if (claimed_index < 0) {
        return;
    }

    // Blocking filesystem write
    LogBuffer *buffer = &buffers[claimed_index];

    // For testing
    uint32_t write_start = HAL_GetTick();

    UINT bytes_written = 0;
    FRESULT result = f_write(
        &log_file,
        buffer->data,
        buffer->used,
        &bytes_written
    );

    // For testing
    uint32_t write_end = HAL_GetTick();
    uint32_t write_time = write_end - write_start;
    last_write_ms = write_time;
    max_write_ms = (write_time > max_write_ms) ? write_time : max_write_ms;
    last_used = buffer->used;

    bool successful = (result == FR_OK) && (bytes_written == buffer->used);

    primask = __get_PRIMASK();
    __disable_irq();

    if(successful) {
        buffer->used = 0;
        ready_index = -1;
    } else {
        storage_error = true;
    }

    writer_busy = false;

    if (primask == 0U) {
        __enable_irq();
    }
}

bool BlackboxLogger_Flush(void) {
    BlackboxLogger_Service();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (storage_error || ready_index >= 0) {
        __set_PRIMASK(primask);
        return false;
    }

    LogBuffer *active = &buffers[active_index];

    if (active->used > 0) {
        ready_index = (int8_t)active_index;
        active_index ^= 1U;
        buffers[active_index].used = 0;
    }

    if(primask == 0U) {
        __enable_irq();
    }

    BlackboxLogger_Service();

    if (storage_error || ready_index >= 0) {
        return false;
    }

    uint32_t sync_start = HAL_GetTick();

    if (f_sync(&log_file) != FR_OK) {
        storage_error = true;
        return false;
    }

    uint32_t sync_end = HAL_GetTick();
    sync_time = sync_end - sync_start;
    max_sync_time = (sync_time > max_sync_time) ? sync_time : max_sync_time;

    return true;
}

uint32_t BlackboxLogger_GetDroppedRecordCount(void) {
    return dropped_records;
}

bool BlackboxLogger_HasStorageError(void) {
    return storage_error;
}
