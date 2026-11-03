#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "wire_protocol.h"
#include "a_uart.h"
#ifndef CONFIG_STYLE_MOCK_SOURCE
#define CONFIG_STYLE_MOCK_SOURCE 0
#endif
#ifndef CONFIG_STYLE_MIC_ENABLE
#define CONFIG_STYLE_MIC_ENABLE 0
#endif
typedef struct {uint32_t frame_id,style_epoch,token,style_crc32,guide_crc32;uint16_t source_fps_x100;uint8_t style_id,effect;bool hd;int64_t deadline_us;} fpga_frame_t;
typedef struct {bool extended,frame_valid,model_valid,target_pending;uint32_t latest_frame_id,active_epoch;uint8_t active_style,target_style,requested_mode,accepted_mode,actual_mode;bool fps_requested,fps_actual,effect_requested,effect_actual;uint8_t source_fps;a_uart_status_t a_state;} fpga_live_status_t;
esp_err_t fpga_transport_init(void);
esp_err_t fpga_transport_acquire(fpga_frame_t *frame,bool hd);
esp_err_t fpga_transport_read_chunk(const fpga_frame_t *frame,uint8_t plane,size_t offset,uint8_t *out,size_t bytes);
esp_err_t fpga_transport_release(const fpga_frame_t *frame);
esp_err_t fpga_transport_query(fpga_frame_t *frame);
esp_err_t fpga_transport_probe(bool *extended,bool *frame_valid,uint32_t *latest_id);
esp_err_t fpga_transport_live(fpga_live_status_t *status);
esp_err_t fpga_transport_remote(uint8_t action,uint8_t value,bool *uncertain);
esp_err_t fpga_transport_remote_style(uint8_t style_id,bool next,bool *uncertain);
esp_err_t fpga_transport_set_style(uint8_t style_id,bool next);
esp_err_t fpga_transport_audio(const int16_t *samples,size_t count,uint32_t sequence);
void fpga_transport_audio_discontinuity(uint32_t samples);
void microphone_start(void);
