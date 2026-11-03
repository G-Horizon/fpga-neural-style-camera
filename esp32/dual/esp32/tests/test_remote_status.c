#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include "freertos/task.h"
static jmp_buf stop_worker;
#define ulTaskNotifyTake(clear,wait) longjmp(stop_worker,1)
#include "../main/remote_debug.c"
#undef ulTaskNotifyTake
int host_async_begun,host_async_completed,host_registered;static bool a_online=true,b_online=false;static unsigned delivered;
esp_err_t a_uart_status(a_uart_status_t*s){*s=(a_uart_status_t){.connected=a_online,.session=0xfedcba98,.requested_style=2,.active_style=2,.requested_epoch=0xabcdef01,.capture_fid=0xffffff01,.computed_fid=0xffffff00,.transmitted_fid=0xfffffeff,.pcm_free=1024,.capabilities=7};return a_online?ESP_OK:ESP_ERR_TIMEOUT;}
esp_err_t fpga_transport_probe(bool*e,bool*v,uint32_t*f){*e=true;*v=b_online;*f=123;return b_online?ESP_OK:ESP_ERR_TIMEOUT;}
esp_err_t fpga_transport_live(fpga_live_status_t*s){*s=(fpga_live_status_t){.extended=true,.model_valid=true,.frame_valid=true,.active_style=3,.active_epoch=0xfedcba98,.source_fps=60,.latest_frame_id=123,.actual_mode=2};return b_online?ESP_OK:ESP_ERR_TIMEOUT;}
esp_err_t fpga_transport_remote_style(uint8_t style,bool next,bool*u){assert(style==0&&next);*u=false;delivered++;return ESP_OK;}
esp_err_t fpga_transport_remote(uint8_t a,uint8_t v,bool*u){(void)a;(void)v;*u=false;return ESP_OK;}
static unsigned viewers(void){return 3;}
int main(void){assert(remote_debug_start(NULL,true,viewers)==ESP_OK);poll_fpga();httpd_req_t r={0};status_handler(&r);assert(strstr(r.response,"\"bridge\":{\"transport_ready\":true,\"connected\":false"));assert(strstr(r.response,"\"a_state\":{\"connected\":true"));assert(strstr(r.response,"\"session\":4275878552"));assert(strstr(r.response,"\"capabilities\":{\"style\":true,\"extended\":true"));
 const char *body="{\"request_id\":\"a-only\",\"command\":\"next\"}";r=(httpd_req_t){.body=body,.content_type="application/json",.content_len=strlen(body)};receive_control(&r);assert(!strncmp(r.status,"202",3));if(!setjmp(stop_worker))control_worker(NULL);assert(delivered==1);r=(httpd_req_t){0};status_handler(&r);assert(strstr(r.response,"\"bridge\":{\"transport_ready\":true,\"connected\":false"));assert(strstr(r.response,"\"state\":\"sent\""));const char*results=getenv("STYLE_TEST_RESULTS");char report_path[1024];snprintf(report_path,sizeof(report_path),"%s/http_a_only.json",results?results:"tests/results");FILE*f=fopen(report_path,"wb");assert(f);fputs(r.response,f);fclose(f);
 b_online=true;poll_fpga();r=(httpd_req_t){0};status_handler(&r);assert(strstr(r.response,"\"bridge\":{\"transport_ready\":true,\"connected\":true"));assert(strstr(r.response,"\"active_epoch\":4275878552"));assert(strstr(r.response,"\"latest_frame_id\":123"));a_online=false;poll_fpga();r=(httpd_req_t){0};status_handler(&r);assert(strstr(r.response,"\"a_state\":{\"connected\":false"));assert(strstr(r.response,"\"capabilities\":{\"style\":false"));
 puts("PASS: actual production HTTPworker/status,A-online/B-offline controls accepted once without claimingBconnected;32bitA/Bsnapshots independent;Aoffline gates controls");return 0;}
