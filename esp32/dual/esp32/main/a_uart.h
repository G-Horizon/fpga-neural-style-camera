#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct {bool connected;uint32_t session;uint8_t status,requested_style,active_style,layout,fps,effect,model_valid,busy;uint32_t requested_epoch,capture_fid,computed_fid,transmitted_fid,next_audio_sample,faults;uint16_t pcm_free,capabilities;} a_uart_status_t;
esp_err_t a_uart_init(void);
esp_err_t a_uart_status(a_uart_status_t *status);
esp_err_t a_uart_control(uint8_t action,uint8_t value,bool *uncertain);
esp_err_t a_uart_audio(const int16_t *samples,size_t count);
void a_uart_discontinuity(uint32_t skipped);
