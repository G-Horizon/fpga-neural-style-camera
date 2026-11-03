#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define D2_UART_VERSION 1
#define D2_UART_MAX_PAYLOAD 128
#define D2_UART_MAX_PACKET 140
enum { D2_HELLO=0,D2_CONTROL=1,D2_STATUS=2,D2_PCM=3,D2_PCM_ABORT=4 };
typedef struct {uint8_t type;uint32_t sequence;uint16_t length;uint8_t payload[D2_UART_MAX_PAYLOAD];} d2_uart_packet_t;
typedef struct {uint8_t data[D2_UART_MAX_PACKET];size_t used;uint32_t last_ms,crc_errors,length_errors,timeouts,discarded;} d2_uart_parser_t;
uint16_t d2_crc16(const uint8_t *p,size_t n);
uint32_t d2_u32(const uint8_t *p);
void d2_put32(uint8_t *p,uint32_t v);
size_t d2_uart_encode(uint8_t *out,size_t capacity,const d2_uart_packet_t *p);
bool d2_uart_feed(d2_uart_parser_t *s,uint8_t byte,uint32_t now_ms,d2_uart_packet_t *out);
typedef struct {uint32_t session,last_id,pending_id;uint8_t action,value;bool pending,uncertain;} d2_control_host_t;
bool d2_control_begin(d2_control_host_t *s,uint8_t action,uint8_t value,uint32_t *id);
void d2_control_complete(d2_control_host_t *s,uint32_t session,uint32_t id);
void d2_control_timeout(d2_control_host_t *s);
void d2_control_new_session(d2_control_host_t *s,uint32_t session);
