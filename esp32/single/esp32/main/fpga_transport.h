#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "wire_protocol.h"
#include "sdkconfig.h"
#ifndef CONFIG_STYLE_MOCK_SOURCE
#define CONFIG_STYLE_MOCK_SOURCE 0
#endif
#ifndef CONFIG_STYLE_MIC_ENABLE
#define CONFIG_STYLE_MIC_ENABLE 0
#endif
typedef struct {
    uint32_t frame_id, style_epoch;
    uint16_t source_fps_x100;
    uint8_t style_id, effect;
    uint32_t token;
} fpga_frame_t;
esp_err_t fpga_transport_init(void);
// Acquire an immutable complete FPGA frame; return NOT_FOUND if none is ready.
esp_err_t fpga_transport_acquire(fpga_frame_t *frame);
// Read a chunk of that acquired frame, in RGB888 R,G,B byte order.
esp_err_t fpga_transport_read_chunk(const fpga_frame_t *frame, size_t offset, uint8_t *out, size_t bytes);
esp_err_t fpga_transport_release(const fpga_frame_t *frame);
esp_err_t fpga_transport_audio(const int16_t *samples, size_t count, uint32_t sequence);
void fpga_transport_audio_discontinuity(uint32_t skipped_samples);
esp_err_t fpga_transport_status(void);
// Query the latest picture and live presentation controls without leasing it.
// The query's token/style_epoch contain the raw FPGA 16-bit style epoch.
esp_err_t fpga_transport_query(fpga_frame_t *frame);
// Physical style IDs 1/2/3; next ignores style_id. Opcode 6 uses flags bit 0.
esp_err_t fpga_transport_set_style(uint8_t style_id, bool next);
typedef struct {
    bool extended, frame_valid, model_valid, target_pending;
    uint32_t latest_frame_id;
    uint16_t active_epoch;
    uint8_t active_style, target_style, requested_mode, accepted_mode, actual_mode;
    bool fps_requested, fps_actual, effect_requested, effect_actual;
    uint8_t source_fps;
} fpga_live_status_t;
// Probe is opcode 1 only: safe with old bitstreams, including no-frame replies.
esp_err_t fpga_transport_probe(bool *extended, bool *frame_valid, uint32_t *latest_id);
// Call only after the D1 capability has been established. No video lease changes.
esp_err_t fpga_transport_live(fpga_live_status_t *status);
// Action 0=query, 1=explicit FPS, 2=explicit mode, 3=next mode, 4=effect.
// uncertain is true only when a CMD was attempted but no definitive ACK/rejection arrived.
esp_err_t fpga_transport_remote(uint8_t action, uint8_t value, bool *uncertain);
esp_err_t fpga_transport_remote_style(uint8_t style_id, bool next, bool *uncertain);
void microphone_start(void);
