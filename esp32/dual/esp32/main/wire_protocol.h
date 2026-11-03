#pragma once
#include <stdint.h>
#define STREAM_WIDTH 256
#define STREAM_HEIGHT 144
#define STREAM_PAYLOAD_BYTES 110592u
#define GUIDE_PAYLOAD_BYTES 2073600u
#define STREAM_MAX_BYTES (STREAM_PAYLOAD_BYTES+GUIDE_PAYLOAD_BYTES)
#define STREAM_MAGIC UINT32_C(0x32463244)
#define CHUNK_MAGIC UINT32_C(0x32433244)
#define END_MAGIC UINT32_C(0x32453244)
#define ACK_MAGIC UINT32_C(0x324b4341)
#define STREAM_CHUNK_BYTES 8192u
typedef struct __attribute__((packed)) {
 uint32_t magic;uint16_t version,header_bytes;uint32_t frame_id,style_epoch,sequence,timestamp_ms;
 uint8_t style_id,effect,quality,flags;uint16_t source_fps_x100,width,height,guide_width,guide_height,algorithm;
 uint32_t style_bytes,guide_bytes,style_crc32,guide_crc32,stream_session,header_crc32;
} stream_header_t;
typedef struct __attribute__((packed)) {uint32_t magic,sequence,frame_id,style_epoch;uint8_t plane,reserved;uint16_t bytes;uint32_t offset,payload_crc32,header_crc32;} stream_chunk_t;
typedef struct __attribute__((packed)) {uint32_t magic,sequence,frame_id,style_epoch,style_crc32,guide_crc32,total_bytes,header_crc32;} stream_end_t;
typedef struct __attribute__((packed)) {uint32_t magic,frame_id,style_epoch,sequence;} stream_ack_t;
_Static_assert(sizeof(stream_header_t)==64,"D2F2 header");
_Static_assert(sizeof(stream_chunk_t)==32,"D2C2 header");
_Static_assert(sizeof(stream_end_t)==32,"D2E2 header");
#define D2_SPI_MAGIC UINT32_C(0x50533244)
#define D2_SPI_VERSION 2
#define D2_SPI_PACKET 512u
#define D2_SPI_HEADER 48u
#define D2_SPI_DATA 464u
enum {D2_SPI_CAPS=0x20,D2_SPI_ACQUIRE=0x21,D2_SPI_READ=0x22,D2_SPI_RELEASE=0x23};
typedef struct __attribute__((packed)) {
 uint32_t magic;uint8_t version,opcode,flags,status;uint32_t sequence,lease,frame_id,epoch,offset;
 uint16_t length;uint8_t plane,style,effect,fps,layout,reserved;uint32_t plane_bytes,payload_crc32,header_crc32;
} d2_spi_header_t;
typedef struct __attribute__((packed)) {uint32_t style_bytes,guide_bytes,style_crc32,guide_crc32;uint16_t guide_width,guide_height,style_width,style_height;uint32_t format_flags,reserved;} d2_spi_meta_t;
_Static_assert(sizeof(d2_spi_header_t)==48,"D2SP header");
_Static_assert(sizeof(d2_spi_meta_t)==32,"D2SP meta");
uint32_t d2_crc32(const uint8_t *bytes,unsigned length);
