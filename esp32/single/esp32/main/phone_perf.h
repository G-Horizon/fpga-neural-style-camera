#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Observers only: durations are wall-clock microseconds, including preemption.
typedef struct {
    uint64_t cmd_us,ready_us,nop_us,crc_us,bus_lock_us,command_total_us;
    uint64_t delay_us,copy_us,driver_bus_us;
    uint32_t commands,command_failures,ready_sleep_count,ready_slept_commands;
    uint32_t ready_max_us;
} phone_spi_perf_t;
typedef struct {
    phone_spi_perf_t spi;
    uint64_t total_us,query_us,acquire_us,read_us,release_us;
    uint32_t frame_id,result;
    int32_t error;
} phone_frame_perf_t;
enum {
    PHONE_PERF_PUBLISHED=0,PHONE_PERF_QUERY_FAILED=1,PHONE_PERF_UNCHANGED=2,
    PHONE_PERF_ACQUIRE_FAILED=3,PHONE_PERF_READ_FAILED=4,
    PHONE_PERF_RELEASE_FAILED=5,PHONE_PERF_INVALID_METADATA=6
};
enum { PHONE_PERF_QUERY=0,PHONE_PERF_ACQUIRE=1,PHONE_PERF_READ=2,PHONE_PERF_RELEASE=3 };
void phone_perf_frame_begin(void);
void phone_perf_frame_phase(unsigned phase,uint64_t elapsed_us);
void phone_perf_frame_end(unsigned result,esp_err_t error,uint32_t frame_id);
// A coherent snapshot; background SPI is in all_spi, not the capture-frame totals.
int phone_perf_json(char *buffer,size_t capacity);
