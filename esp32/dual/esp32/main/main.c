#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "fpga_transport.h"
#include "remote_debug.h"
#include "frame_pool.h"
#define CLIENTS 3
#define SEND_TIMEOUT_US INT64_C(10000000)
#define ACK_TIMEOUT_US INT64_C(14000000)
typedef struct {bool active,hd;int fd;TaskHandle_t task;d2_frame_credit_t flow;} client_t;
static client_t clients[CLIENTS];static SemaphoreHandle_t lock;static httpd_handle_t server;
/* Exactly one image allocation, independent of viewer count. No client copies a frame. */
static uint8_t *pool;static d2_frame_pool_t ownership;static stream_header_t current;static uint32_t session;
static void session_close(httpd_handle_t hd,int fd){(void)hd;bool owned=false;xSemaphoreTake(lock,portMAX_DELAY);for(int i=0;i<CLIENTS;i++)if(clients[i].task&&clients[i].fd==fd){clients[i].active=false;owned=true;xTaskNotifyGive(clients[i].task);break;}xSemaphoreGive(lock);shutdown(fd,SHUT_RDWR);if(!owned)close(fd);}
static esp_err_t socket_bytes(int fd,const uint8_t *p,size_t n,int64_t deadline){while(n){if(esp_timer_get_time()>deadline)return ESP_ERR_TIMEOUT;int sent=send(fd,p,n,0);if(sent<=0)return ESP_FAIL;p+=sent;n-=(size_t)sent;}return ESP_OK;}
static esp_err_t binary(int fd,const void *head,size_t hs,const void *data,size_t n,int64_t deadline){uint8_t w[10]={0x82,0};uint64_t length=hs+n;size_t wh;
 if(length<126){w[1]=(uint8_t)length;wh=2;}else if(length<=65535){w[1]=126;w[2]=(uint8_t)(length>>8);w[3]=(uint8_t)length;wh=4;}else{w[1]=127;for(int i=0;i<8;i++)w[9-i]=(uint8_t)(length>>(i*8));wh=10;}
 esp_err_t e=socket_bytes(fd,w,wh,deadline);if(e==ESP_OK)e=socket_bytes(fd,head,hs,deadline);if(e==ESP_OK&&n)e=socket_bytes(fd,data,n,deadline);return e;}
static esp_err_t send_frame(client_t *c,stream_header_t h){
 if(!c->hd){h.quality=0;h.guide_width=h.guide_height=h.algorithm=0;h.guide_bytes=h.guide_crc32=0;}h.header_crc32=d2_crc32((const uint8_t*)&h,60);
 int64_t end=esp_timer_get_time()+SEND_TIMEOUT_US;esp_err_t e=binary(c->fd,&h,64,NULL,0,end);
 for(uint8_t plane=0;plane<2&&e==ESP_OK;plane++){size_t bytes=plane?h.guide_bytes:h.style_bytes;const uint8_t *base=pool+(plane?STREAM_PAYLOAD_BYTES:0);
  for(size_t offset=0;offset<bytes&&e==ESP_OK;offset+=STREAM_CHUNK_BYTES){size_t n=bytes-offset;if(n>STREAM_CHUNK_BYTES)n=STREAM_CHUNK_BYTES;
   stream_chunk_t chunk={.magic=CHUNK_MAGIC,.sequence=h.sequence,.frame_id=h.frame_id,.style_epoch=h.style_epoch,.plane=plane,.bytes=(uint16_t)n,.offset=(uint32_t)offset,.payload_crc32=d2_crc32(base+offset,(unsigned)n)};chunk.header_crc32=d2_crc32((const uint8_t*)&chunk,28);e=binary(c->fd,&chunk,32,base+offset,n,end);
  }
 }
 if(e==ESP_OK){stream_end_t t={.magic=END_MAGIC,.sequence=h.sequence,.frame_id=h.frame_id,.style_epoch=h.style_epoch,.style_crc32=h.style_crc32,.guide_crc32=h.guide_crc32,.total_bytes=h.style_bytes+h.guide_bytes};t.header_crc32=d2_crc32((const uint8_t*)&t,28);e=binary(c->fd,&t,32,NULL,0,end);}return e;
}
static void client_sender(void *arg){client_t *c=arg;for(;;){ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));bool send_now=false,expire=false;stream_header_t h;
 xSemaphoreTake(lock,portMAX_DELAY);if(!c->active){xSemaphoreGive(lock);break;}expire=d2_credit_expired(&c->flow,esp_timer_get_time(),ACK_TIMEOUT_US);
 if(d2_credit_eligible(&c->flow,current.frame_id,current.style_epoch,current.sequence)&&d2_pool_acquire(&ownership)){h=current;bool began=d2_credit_begin(&c->flow,h.frame_id,h.style_epoch,h.sequence,esp_timer_get_time());assert(began);(void)began;send_now=true;}xSemaphoreGive(lock);if(expire)break;
 if(send_now){esp_err_t e=send_frame(c,h);xSemaphoreTake(lock,portMAX_DELAY);bool released=d2_pool_release(&ownership);assert(released);(void)released;xSemaphoreGive(lock);if(e!=ESP_OK)break;}
 }
 int fd=c->fd;xSemaphoreTake(lock,portMAX_DELAY);bool live=c->active;xSemaphoreGive(lock);if(live)httpd_sess_trigger_close(server,fd);
 for(;;){xSemaphoreTake(lock,portMAX_DELAY);live=c->active;xSemaphoreGive(lock);if(!live)break;ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));}close(fd);xSemaphoreTake(lock,portMAX_DELAY);c->task=NULL;xSemaphoreGive(lock);vTaskDelete(NULL);
}
static esp_err_t stream(httpd_req_t *r){int fd=httpd_req_to_sockfd(r);if(r->method==HTTP_GET){char query[80]={0},quality[12]={0};bool hd=false;if(httpd_req_get_url_query_str(r,query,sizeof(query))==ESP_OK&&httpd_query_key_value(query,"quality",quality,sizeof(quality))==ESP_OK)hd=!strcmp(quality,"hd");
 xSemaphoreTake(lock,portMAX_DELAY);client_t *c=NULL;for(int i=0;i<CLIENTS;i++)if(!clients[i].active&&!clients[i].task){c=&clients[i];break;}
 if(c){*c=(client_t){.active=true,.hd=hd,.fd=fd,.flow={.credit=true,.id=UINT32_MAX}};if(xTaskCreate(client_sender,"ws_client",4096,c,3,&c->task)!=pdPASS){c->active=false;c=NULL;}}xSemaphoreGive(lock);if(c){struct timeval t={.tv_sec=2};setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&t,sizeof(t));}return c?ESP_OK:ESP_FAIL;}
 httpd_ws_frame_t f={0};esp_err_t e=httpd_ws_recv_frame(r,&f,0);if(e!=ESP_OK)return e;if(f.type!=HTTPD_WS_TYPE_BINARY||f.len!=16)return ESP_FAIL;stream_ack_t a;f.payload=(uint8_t*)&a;e=httpd_ws_recv_frame(r,&f,16);if(e!=ESP_OK||a.magic!=ACK_MAGIC)return ESP_FAIL;
 xSemaphoreTake(lock,portMAX_DELAY);for(int i=0;i<CLIENTS;i++){client_t *c=&clients[i];if(c->active&&c->fd==fd&&d2_credit_ack(&c->flow,a.frame_id,a.style_epoch,a.sequence)){xTaskNotifyGive(c->task);break;}}xSemaphoreGive(lock);return ESP_OK;
}
extern const uint8_t index_start[] asm("_binary_index_html_start"),index_end[] asm("_binary_index_html_end");
extern const uint8_t css_start[] asm("_binary_style_css_start"),css_end[] asm("_binary_style_css_end");
extern const uint8_t app_start[] asm("_binary_app_mjs_start"),app_end[] asm("_binary_app_mjs_end");
extern const uint8_t proto_start[] asm("_binary_protocol_mjs_start"),proto_end[] asm("_binary_protocol_mjs_end");
extern const uint8_t debug_start[] asm("_binary_remote_debug_html_start"),debug_end[] asm("_binary_remote_debug_html_end");
extern const uint8_t debug_css_start[] asm("_binary_remote_debug_css_start"),debug_css_end[] asm("_binary_remote_debug_css_end");
extern const uint8_t debug_app_start[] asm("_binary_remote_debug_mjs_start"),debug_app_end[] asm("_binary_remote_debug_mjs_end");
extern const uint8_t debug_api_start[] asm("_binary_remote_api_mjs_start"),debug_api_end[] asm("_binary_remote_api_mjs_end");
static esp_err_t asset(httpd_req_t *r){const uint8_t *a=index_start,*b=index_end;const char *type="text/html; charset=utf-8";
 if(!strcmp(r->uri,"/style.css")){a=css_start;b=css_end;type="text/css";}else if(!strcmp(r->uri,"/app.mjs")){a=app_start;b=app_end;type="text/javascript";}else if(!strcmp(r->uri,"/protocol.mjs")){a=proto_start;b=proto_end;type="text/javascript";}else if(!strcmp(r->uri,"/debug")||!strcmp(r->uri,"/debug/")){a=debug_start;b=debug_end;}else if(!strcmp(r->uri,"/debug/remote_debug.css")){a=debug_css_start;b=debug_css_end;type="text/css";}else if(!strcmp(r->uri,"/debug/remote_debug.mjs")){a=debug_app_start;b=debug_app_end;type="text/javascript";}else if(!strcmp(r->uri,"/debug/remote_api.mjs")){a=debug_api_start;b=debug_api_end;type="text/javascript";}
 httpd_resp_set_type(r,type);httpd_resp_set_hdr(r,"Cache-Control","no-cache");httpd_resp_set_hdr(r,"Connection","close");return httpd_resp_send(r,(const char*)a,b-a-1);
}
static void capture(void *unused){(void)unused;uint32_t seq=0;int64_t next=0;bool disconnected=false;
 for(;;){vTaskDelay(pdMS_TO_TICKS(10));if(esp_timer_get_time()<next)continue;bool any=false,hd=false;stream_header_t old;
  xSemaphoreTake(lock,portMAX_DELAY);for(int i=0;i<CLIENTS;i++)if(clients[i].active){any=true;hd|=clients[i].hd;}old=current;bool available=ownership.refs==0&&!ownership.capturing;xSemaphoreGive(lock);if(!any||!available)continue;
  int64_t start=esp_timer_get_time();next=start+(hd?2000000:125000);
  if(!CONFIG_STYLE_MOCK_SOURCE){fpga_frame_t q;esp_err_t e=fpga_transport_query(&q);if(e!=ESP_OK){disconnected=true;continue;}if(!disconnected&&old.magic==STREAM_MAGIC&&old.frame_id==q.frame_id&&old.style_epoch==q.style_epoch&&old.effect==q.effect&&old.quality==(uint8_t)hd)continue;}
  xSemaphoreTake(lock,portMAX_DELAY);bool began=d2_pool_begin(&ownership);xSemaphoreGive(lock);if(!began)continue;
  fpga_frame_t f;esp_err_t e=fpga_transport_acquire(&f,hd);bool leased=e==ESP_OK;if(leased)f.deadline_us=start+4500000;
  for(uint8_t plane=0;plane<2&&e==ESP_OK;plane++){size_t bytes=plane?(hd?GUIDE_PAYLOAD_BYTES:0):STREAM_PAYLOAD_BYTES;for(size_t offset=0;offset<bytes&&e==ESP_OK;offset+=STREAM_CHUNK_BYTES){if(esp_timer_get_time()-start>4500000){e=ESP_ERR_TIMEOUT;break;}size_t n=bytes-offset;if(n>STREAM_CHUNK_BYTES)n=STREAM_CHUNK_BYTES;e=fpga_transport_read_chunk(&f,plane,offset,pool+(plane?STREAM_PAYLOAD_BYTES:0)+offset,n);}}
  uint32_t sc=0,gc=0;if(e==ESP_OK){sc=d2_crc32(pool,STREAM_PAYLOAD_BYTES);gc=hd?d2_crc32(pool+STREAM_PAYLOAD_BYTES,GUIDE_PAYLOAD_BYTES):0;if(!CONFIG_STYLE_MOCK_SOURCE&&(sc!=f.style_crc32||gc!=f.guide_crc32))e=ESP_ERR_INVALID_CRC;}
  if(leased){esp_err_t re=fpga_transport_release(&f);if(e==ESP_OK)e=re;}
  /* A/B can reboot between two successful CAPS polls. A complete snapshot is
   * the authority; a partial read/failed RELEASE must never renew the stream.
   * Keep sequence monotonic across sessions. Waiting old ACK credit stays
   * retired normally before that client can start the new session. */
  stream_header_t h={0};if(e==ESP_OK){bool regressed=old.magic==STREAM_MAGIC&&d2_stream_identity_regressed(old.frame_id,old.style_epoch,f.frame_id,f.style_epoch);if(disconnected||regressed){uint32_t previous=session;do{session=esp_random();}while(!session||session==previous);disconnected=false;}h=(stream_header_t){.magic=STREAM_MAGIC,.version=2,.header_bytes=64,.frame_id=f.frame_id,.style_epoch=f.style_epoch,.sequence=++seq,.timestamp_ms=(uint32_t)(esp_timer_get_time()/1000),.style_id=f.style_id,.effect=f.effect,.quality=hd,.flags=3|(CONFIG_STYLE_MOCK_SOURCE?4:0),.source_fps_x100=f.source_fps_x100,.width=256,.height=144,.guide_width=hd?1920:0,.guide_height=hd?1080:0,.algorithm=hd?1:0,.style_bytes=STREAM_PAYLOAD_BYTES,.guide_bytes=hd?GUIDE_PAYLOAD_BYTES:0,.style_crc32=sc,.guide_crc32=gc,.stream_session=session};h.header_crc32=d2_crc32((const uint8_t*)&h,60);}else disconnected=true;
  xSemaphoreTake(lock,portMAX_DELAY);bool finished=d2_pool_publish(&ownership,e==ESP_OK);assert(finished);(void)finished;if(e==ESP_OK){current=h;for(int i=0;i<CLIENTS;i++)if(clients[i].active)xTaskNotifyGive(clients[i].task);}xSemaphoreGive(lock);
  int64_t elapsed=esp_timer_get_time()-start,base=hd?2000000:125000;next=start+(elapsed>base?elapsed+10000:base);
 }
}
static unsigned active_stream_clients(void){unsigned n=0;xSemaphoreTake(lock,portMAX_DELAY);for(int i=0;i<CLIENTS;i++)if(clients[i].active)n++;xSemaphoreGive(lock);return n;}
void app_main(void){esp_err_t e=nvs_flash_init();if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();}ESP_ERROR_CHECK(e);lock=xSemaphoreCreateMutex();assert(lock);pool=heap_caps_malloc(STREAM_MAX_BYTES,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);assert(pool);do{session=esp_random();}while(!session);
 ESP_ERROR_CHECK(esp_netif_init());ESP_ERROR_CHECK(esp_event_loop_create_default());esp_netif_create_default_wifi_ap();wifi_init_config_t wc=WIFI_INIT_CONFIG_DEFAULT();ESP_ERROR_CHECK(esp_wifi_init(&wc));wifi_config_t ap={.ap={.channel=6,.max_connection=4,.authmode=WIFI_AUTH_WPA2_PSK}};strlcpy((char*)ap.ap.ssid,CONFIG_STYLE_AP_SSID,sizeof(ap.ap.ssid));ap.ap.ssid_len=strlen(CONFIG_STYLE_AP_SSID);strlcpy((char*)ap.ap.password,CONFIG_STYLE_AP_PASSWORD,sizeof(ap.ap.password));ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));ESP_ERROR_CHECK(esp_wifi_start());
 httpd_config_t hc=HTTPD_DEFAULT_CONFIG();hc.max_open_sockets=9;hc.max_uri_handlers=16;hc.stack_size=12288;hc.send_wait_timeout=2;hc.recv_wait_timeout=2;hc.lru_purge_enable=false;hc.close_fn=session_close;ESP_ERROR_CHECK(httpd_start(&server,&hc));const char *paths[]={"/","/style.css","/app.mjs","/protocol.mjs","/debug","/debug/","/debug/remote_debug.css","/debug/remote_debug.mjs","/debug/remote_api.mjs"};for(size_t i=0;i<sizeof(paths)/sizeof(paths[0]);i++){httpd_uri_t u={.uri=paths[i],.method=HTTP_GET,.handler=asset};ESP_ERROR_CHECK(httpd_register_uri_handler(server,&u));}httpd_uri_t ws={.uri="/stream",.method=HTTP_GET,.handler=stream,.is_websocket=true,.handle_ws_control_frames=true};ESP_ERROR_CHECK(httpd_register_uri_handler(server,&ws));
 e=fpga_transport_init();if(e==ESP_OK){microphone_start();ESP_ERROR_CHECK(xTaskCreate(capture,"capture",4096,NULL,4,NULL)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);}ESP_ERROR_CHECK(remote_debug_start(server,e==ESP_OK,active_stream_clients));ESP_LOGI("dualboard","%s: http://192.168.4.1 /debug; 3shared viewers,one%uB PSRAM image; source=%s",CONFIG_STYLE_AP_SSID,(unsigned)STREAM_MAX_BYTES,CONFIG_STYLE_MOCK_SOURCE?"SIMULATION":"physicalB-SPI/A-UART");
}
