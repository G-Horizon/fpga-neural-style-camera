#pragma once
#include <stdint.h>
#define STREAM_WIDTH 256
#define STREAM_HEIGHT 144
#define STREAM_PAYLOAD_BYTES (STREAM_WIDTH * STREAM_HEIGHT * 3)
#define STREAM_MAGIC UINT32_C(0x31525346)
#define ACK_MAGIC UINT32_C(0x314b4341)
// Packed little-endian protocol. ESP32-S3 is little endian; browser uses DataView.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version, header_bytes;
    uint32_t frame_id, style_epoch, timestamp_ms;
    uint8_t style_id, effect;
    uint16_t flags, width, height, stride;
    uint8_t pixel_format, reserved0;
    uint32_t payload_bytes;
    uint16_t source_fps_x100, reserved1;
    uint32_t payload_crc32, sequence;
} stream_header_t;
typedef struct __attribute__((packed)) { uint32_t magic, frame_id, style_epoch, sequence; } stream_ack_t;
_Static_assert(sizeof(stream_header_t)==48,"wire header layout");
_Static_assert(sizeof(stream_ack_t)==16,"ACK layout");
