#pragma once
#include "host_stubs.h"
#define UART_NUM_1 1
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
typedef struct {int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk;}uart_config_t;
esp_err_t uart_param_config(int port,const uart_config_t *c);
esp_err_t uart_set_pin(int port,int tx,int rx,int rts,int cts);
esp_err_t uart_driver_install(int port,int rx,int tx,int depth,void *queue,int flags);
int uart_write_bytes(int port,const void *p,size_t n);
int uart_read_bytes(int port,void *p,uint32_t n,unsigned ticks);
esp_err_t uart_get_buffered_data_len(int port,size_t *n);
