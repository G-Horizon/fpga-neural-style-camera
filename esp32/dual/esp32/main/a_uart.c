#include "a_uart.h"
#include "a_uart_wire.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <string.h>
#ifndef CONFIG_STYLE_MOCK_SOURCE
#define CONFIG_STYLE_MOCK_SOURCE 0
#endif
static SemaphoreHandle_t mutex;static d2_uart_parser_t parser;static d2_control_host_t host;
static uint32_t sequence,audio_index,skipped;static bool need_abort,definitive_reply;static a_uart_status_t latest;
static uint32_t next_seq(void){if(!++sequence)++sequence;return sequence;}
static void status_decode(const uint8_t *p){latest=(a_uart_status_t){.connected=true,.session=d2_u32(p),.status=p[4],.requested_style=p[5],.active_style=p[6],.layout=p[7],.fps=p[8],.effect=p[9],.model_valid=p[10],.busy=p[11],.requested_epoch=d2_u32(p+12),.capture_fid=d2_u32(p+16),.computed_fid=d2_u32(p+20),.transmitted_fid=d2_u32(p+24),.next_audio_sample=d2_u32(p+28),.faults=d2_u32(p+32),.pcm_free=(uint16_t)(p[36]|p[37]<<8),.capabilities=(uint16_t)(p[38]|p[39]<<8)};}
static esp_err_t exchange(d2_uart_packet_t *p){
 definitive_reply=false;
 uint8_t bytes[D2_UART_MAX_PACKET];size_t n=d2_uart_encode(bytes,sizeof(bytes),p);if(!n)return ESP_ERR_INVALID_ARG;
 if(CONFIG_STYLE_MOCK_SOURCE){definitive_reply=true;latest.connected=true;latest.session=host.session;latest.status=0;if(p->type==D2_CONTROL){uint8_t a=p->payload[4],v=p->payload[5];if(a==1)latest.requested_style=latest.active_style=v;if(a==2)latest.requested_style=latest.active_style=(latest.requested_style+1)%3;if(a<=2)latest.requested_epoch++;if(a==3)latest.layout=v;if(a==4)latest.layout=(latest.layout+1)%3;if(a==5)latest.fps=v;if(a==6)latest.effect=1;}return ESP_OK;}
 if(uart_write_bytes(UART_NUM_1,bytes,n)!=(int)n)return ESP_FAIL;
 int64_t end=esp_timer_get_time()+250000;uint8_t input[160];
 while(esp_timer_get_time()<end){int got=uart_read_bytes(UART_NUM_1,input,1,pdMS_TO_TICKS(2));if(got<0)break;
  /* ACK is52bytes, not160: waiting for a160byte read timeout per PCM
     packet would cap audio below16k samples/s. Drain only bytes present. */
  if(got){size_t available=0;if(uart_get_buffered_data_len(UART_NUM_1,&available)==ESP_OK&&available){if(available>sizeof(input)-1)available=sizeof(input)-1;int extra=uart_read_bytes(UART_NUM_1,input+1,available,0);if(extra>0)got+=extra;}}
  for(int i=0;i<got;i++){d2_uart_packet_t r;if(!d2_uart_feed(&parser,input[i],(uint32_t)(esp_timer_get_time()/1000),&r))continue;
   if(r.type!=(uint8_t)(p->type|0x80)||r.sequence!=p->sequence||r.length!=40)continue;
   if(r.payload[4]>5||r.payload[5]>2||(r.payload[6]>2&&r.payload[6]!=0xff)||r.payload[7]>2||r.payload[8]>1||r.payload[9]>1||r.payload[10]>1||r.payload[11]>1||(r.payload[10]&&r.payload[6]>2)){latest.connected=false;return ESP_ERR_INVALID_RESPONSE;}
   status_decode(r.payload);if(latest.session!=host.session)return ESP_ERR_INVALID_STATE;definitive_reply=true;
   if(latest.status==0){return ESP_OK;}
   return latest.status==2?ESP_ERR_INVALID_STATE:latest.status==3||latest.status==5?ESP_ERR_NO_MEM:ESP_ERR_INVALID_RESPONSE;
  }
 }
 latest.connected=false;return ESP_ERR_TIMEOUT;
}
static esp_err_t bind_session(void){uint32_t s;do{s=esp_random();}while(!s);d2_control_new_session(&host,s);audio_index=0;need_abort=true;skipped=0;d2_uart_packet_t p={.type=D2_HELLO,.sequence=next_seq(),.length=4};d2_put32(p.payload,s);return exchange(&p);}
esp_err_t a_uart_init(void){
 mutex=xSemaphoreCreateMutex();if(!mutex)return ESP_ERR_NO_MEM;
 if(!CONFIG_STYLE_MOCK_SOURCE){uart_config_t c={.baud_rate=1000000,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};ESP_ERROR_CHECK(uart_param_config(UART_NUM_1,&c));ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,4,5,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1,4096,0,0,NULL,0));}
 xSemaphoreTake(mutex,portMAX_DELAY);(void)bind_session();xSemaphoreGive(mutex);return ESP_OK; /* AP remains available while A is offline. */
}
esp_err_t a_uart_status(a_uart_status_t *s){
 if(!mutex){return ESP_ERR_INVALID_STATE;}
 if(xSemaphoreTake(mutex,pdMS_TO_TICKS(300))!=pdTRUE){return ESP_ERR_TIMEOUT;}
 d2_uart_packet_t p={.type=D2_STATUS,.sequence=next_seq(),.length=4};d2_put32(p.payload,host.session);esp_err_t e=exchange(&p);
 if(e==ESP_ERR_INVALID_STATE||!host.session){e=bind_session();if(e==ESP_OK){p.sequence=next_seq();d2_put32(p.payload,host.session);e=exchange(&p);}}
 if(s){*s=latest;}
 xSemaphoreGive(mutex);return e;
}
esp_err_t a_uart_control(uint8_t a,uint8_t v,bool *uncertain){
 if(uncertain){*uncertain=false;}
 if(!mutex){return ESP_ERR_INVALID_STATE;}
 if(xSemaphoreTake(mutex,pdMS_TO_TICKS(300))!=pdTRUE)return ESP_ERR_TIMEOUT;
 uint32_t id;if(!d2_control_begin(&host,a,v,&id)){xSemaphoreGive(mutex);return ESP_ERR_INVALID_ARG;}
 d2_uart_packet_t p={.type=D2_CONTROL,.sequence=id,.length=6};d2_put32(p.payload,host.session);p.payload[4]=a;p.payload[5]=v;esp_err_t e=exchange(&p);
 /* A valid40byte rejection is definitive; timeout/corruption/lost session is uncertain. No command retry or HELLO+replay. */
 bool unknown=!definitive_reply;
 if(unknown)d2_control_timeout(&host);else d2_control_complete(&host,host.session,id);
 if(uncertain){*uncertain=unknown;}
 xSemaphoreGive(mutex);return e;
}
void a_uart_discontinuity(uint32_t n){if(!mutex)return;xSemaphoreTake(mutex,portMAX_DELAY);audio_index+=n;skipped+=n;need_abort=true;xSemaphoreGive(mutex);}
esp_err_t a_uart_audio(const int16_t *s,size_t count){
 if(!mutex||!s){return ESP_ERR_INVALID_ARG;}
 xSemaphoreTake(mutex,portMAX_DELAY);esp_err_t e=ESP_OK;uint32_t unsuccessful=0;
 if(need_abort){d2_uart_packet_t p={.type=D2_PCM_ABORT,.sequence=next_seq(),.length=8};d2_put32(p.payload,host.session);d2_put32(p.payload+4,skipped);e=exchange(&p);if(e==ESP_OK){need_abort=false;skipped=0;}}
 while(count&&e==ESP_OK){size_t n=count>60?60:count;d2_uart_packet_t p={.type=D2_PCM,.sequence=next_seq(),.length=(uint16_t)(8+2*n)};d2_put32(p.payload,host.session);d2_put32(p.payload+4,audio_index);for(size_t i=0;i<n;i++){p.payload[8+2*i]=(uint8_t)s[i];p.payload[9+2*i]=(uint8_t)((uint16_t)s[i]>>8);}e=exchange(&p);if(e!=ESP_OK)unsuccessful=(uint32_t)n;audio_index+=(uint32_t)n;s+=n;count-=n;}
 if(e!=ESP_OK){audio_index+=(uint32_t)count;skipped+=(uint32_t)count+unsuccessful;need_abort=true;}xSemaphoreGive(mutex);return e;
}
