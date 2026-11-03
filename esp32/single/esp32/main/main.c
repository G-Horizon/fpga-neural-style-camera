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
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "fpga_transport.h"
#include "phone_perf.h"
#include "remote_debug.h"

#define CLIENTS 3
#define AP_ASSOCIATIONS 4
#define FRAME_SLOTS (CLIENTS+2)
#define FRAME_BYTES (sizeof(stream_header_t)+STREAM_PAYLOAD_BYTES)
#define ACK_TIMEOUT_MS 5000
#define CAPTURE_FPS (CONFIG_STYLE_MOCK_SOURCE?CONFIG_STYLE_STREAM_FPS:CONFIG_STYLE_PHYSICAL_FPS)
typedef struct {uint8_t *bytes;unsigned refs;} frame_slot_t;
typedef struct {bool active,credit;int fd;TaskHandle_t task;uint32_t id,epoch,sequence;int64_t sent_us;} client_t;
static frame_slot_t slots[FRAME_SLOTS];static client_t clients[CLIENTS];
static SemaphoreHandle_t lock;static int latest=-1;static httpd_handle_t server;
static const char *TAG="style_bridge";
// A closed HTTP session leaves its fd reserved until its sender exits. This
// prevents a blocked sender from accidentally writing to a newly reused fd.
static void session_close(httpd_handle_t hd,int fd){
    (void)hd;bool owned=false;
    xSemaphoreTake(lock,portMAX_DELAY);
    for(int i=0;i<CLIENTS;i++)if(clients[i].task && clients[i].fd==fd){clients[i].active=false;owned=true;xTaskNotifyGive(clients[i].task);break;}
    xSemaphoreGive(lock);shutdown(fd,SHUT_RDWR);if(!owned)close(fd);
}
static esp_err_t socket_bytes(int fd,const uint8_t *p,size_t n,int64_t deadline){
    while(n){if(esp_timer_get_time()>deadline)return ESP_ERR_TIMEOUT;int sent=send(fd,p,n,0);if(sent<=0)return ESP_FAIL;p+=sent;n-=sent;}return ESP_OK;
}
static esp_err_t send_binary(int fd,const uint8_t *p,size_t n){
    // One unmasked FIN binary WebSocket frame, 64-bit network-order length.
    uint8_t h[10]={0x82,127};uint64_t length=n;for(int i=0;i<8;i++)h[9-i]=(uint8_t)(length>>(i*8));
    const int64_t deadline=esp_timer_get_time()+2000000;
    esp_err_t e=socket_bytes(fd,h,sizeof(h),deadline);return e==ESP_OK?socket_bytes(fd,p,n,deadline):e;
}
extern const uint8_t index_start[] asm("_binary_index_html_start"),index_end[] asm("_binary_index_html_end");
extern const uint8_t css_start[] asm("_binary_style_css_start"),css_end[] asm("_binary_style_css_end");
extern const uint8_t app_start[] asm("_binary_app_mjs_start"),app_end[] asm("_binary_app_mjs_end");
extern const uint8_t proto_start[] asm("_binary_protocol_mjs_start"),proto_end[] asm("_binary_protocol_mjs_end");
extern const uint8_t debug_start[] asm("_binary_remote_debug_html_start"),debug_end[] asm("_binary_remote_debug_html_end");
extern const uint8_t debug_css_start[] asm("_binary_remote_debug_css_start"),debug_css_end[] asm("_binary_remote_debug_css_end");
extern const uint8_t debug_app_start[] asm("_binary_remote_debug_mjs_start"),debug_app_end[] asm("_binary_remote_debug_mjs_end");
extern const uint8_t debug_api_start[] asm("_binary_remote_api_mjs_start"),debug_api_end[] asm("_binary_remote_api_mjs_end");
static esp_err_t asset(httpd_req_t *r){
    const uint8_t *a=index_start,*b=index_end;const char *type="text/html; charset=utf-8";
    if(!strcmp(r->uri,"/style.css")){a=css_start;b=css_end;type="text/css";}
    else if(!strcmp(r->uri,"/app.mjs")){a=app_start;b=app_end;type="text/javascript";}
    else if(!strcmp(r->uri,"/protocol.mjs")){a=proto_start;b=proto_end;type="text/javascript";}
    else if(!strcmp(r->uri,"/debug") || !strcmp(r->uri,"/debug/")){a=debug_start;b=debug_end;}
    else if(!strcmp(r->uri,"/debug/remote_debug.css")){a=debug_css_start;b=debug_css_end;type="text/css";}
    else if(!strcmp(r->uri,"/debug/remote_debug.mjs")){a=debug_app_start;b=debug_app_end;type="text/javascript";}
    else if(!strcmp(r->uri,"/debug/remote_api.mjs")){a=debug_api_start;b=debug_api_end;type="text/javascript";}
    httpd_resp_set_type(r,type);httpd_resp_set_hdr(r,"Cache-Control","no-cache");
    // Release short asset connections so three browsers cannot exhaust the
    // HTTP socket budget with idle keep-alive module/CSS connections.
    httpd_resp_set_hdr(r,"Connection","close");return httpd_resp_send(r,(const char*)a,b-a-1);
}
static void client_sender(void *arg){
    client_t *c=arg;
    for(;;){ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));int slot=-1;bool close=false;
      xSemaphoreTake(lock,portMAX_DELAY);
      if(!c->active){xSemaphoreGive(lock);break;}
      close=!c->credit && esp_timer_get_time()-c->sent_us>ACK_TIMEOUT_MS*1000LL;
      if(c->credit && latest>=0){stream_header_t *h=(stream_header_t*)slots[latest].bytes;
        if(c->id!=h->frame_id || c->epoch!=h->style_epoch || c->sequence!=h->sequence){slot=latest;slots[slot].refs++;c->id=h->frame_id;c->epoch=h->style_epoch;c->sequence=h->sequence;c->credit=false;c->sent_us=esp_timer_get_time();}}
      xSemaphoreGive(lock);
      if(close)break;
      if(slot>=0){esp_err_t e=send_binary(c->fd,slots[slot].bytes,FRAME_BYTES);
        xSemaphoreTake(lock,portMAX_DELAY);slots[slot].refs--;xSemaphoreGive(lock);
        if(e!=ESP_OK)break;
      }
    }
    // The slot remains occupied until this sender exits; a reused fd cannot inherit its state.
    int fd=c->fd;
    xSemaphoreTake(lock,portMAX_DELAY);bool live=c->active;xSemaphoreGive(lock);
    if(live)httpd_sess_trigger_close(server,fd);
    for(;;){xSemaphoreTake(lock,portMAX_DELAY);live=c->active;xSemaphoreGive(lock);if(!live)break;ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));}
    close(fd);xSemaphoreTake(lock,portMAX_DELAY);c->task=NULL;xSemaphoreGive(lock);vTaskDelete(NULL);
}
// ESP-IDF 6 completes the upgrade without invoking the normal URI handler.
// Only this post-handshake callback owns client admission and sender creation.
static esp_err_t stream_open(httpd_req_t *r){
    int fd=httpd_req_to_sockfd(r);
    xSemaphoreTake(lock,portMAX_DELAY);client_t *c=NULL;
    for(int i=0;i<CLIENTS;i++)if(!clients[i].active && !clients[i].task){c=&clients[i];break;}
    if(c){*c=(client_t){.active=true,.credit=true,.fd=fd,.id=UINT32_MAX};
      if(xTaskCreate(client_sender,"ws_client",4096,c,3,&c->task)!=pdPASS){c->active=false;c=NULL;}}
    xSemaphoreGive(lock);
    if(c){struct timeval timeout={.tv_sec=2};setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
      ESP_LOGI(TAG,"WebSocket stream client admitted fd=%d",fd);}
    else ESP_LOGW(TAG,"WebSocket stream client unavailable fd=%d",fd);
    return c?ESP_OK:ESP_FAIL;
}
static esp_err_t stream(httpd_req_t *r){
    int fd=httpd_req_to_sockfd(r);
    httpd_ws_frame_t f={0};esp_err_t e=httpd_ws_recv_frame(r,&f,0);if(e!=ESP_OK)return e;
    if(f.type!=HTTPD_WS_TYPE_BINARY || f.len!=sizeof(stream_ack_t))return ESP_FAIL;
    stream_ack_t a;f.payload=(uint8_t*)&a;e=httpd_ws_recv_frame(r,&f,sizeof(a));if(e!=ESP_OK)return e;
    if(a.magic!=ACK_MAGIC)return ESP_FAIL;
    xSemaphoreTake(lock,portMAX_DELAY);
    for(int i=0;i<CLIENTS;i++){client_t *c=&clients[i];if(c->active && c->fd==fd && c->id==a.frame_id && c->epoch==a.style_epoch && c->sequence==a.sequence){c->credit=true;xTaskNotifyGive(c->task);break;}}
    xSemaphoreGive(lock);return ESP_OK;
}
// Diagnostic only: no SPI transaction is performed by this HTTP handler.
static esp_err_t perf_handler(httpd_req_t *r){
    char *body=malloc(4096);if(!body)return ESP_ERR_NO_MEM;
    int n=phone_perf_json(body,4096);esp_err_t e=ESP_FAIL;
    if(n>=0){httpd_resp_set_type(r,"application/json");httpd_resp_set_hdr(r,"Cache-Control","no-store");e=httpd_resp_send(r,body,n);}
    free(body);return e;
}
static void capture(void *arg){
    (void)arg;TickType_t next=xTaskGetTickCount();uint32_t sequence=0;
    for(;;){vTaskDelayUntil(&next,pdMS_TO_TICKS(1000/CAPTURE_FPS));
      int selected=-1;bool any=false;
      xSemaphoreTake(lock,portMAX_DELAY);
      for(int i=0;i<CLIENTS;i++)any|=clients[i].active;
      if(any)for(int i=0;i<FRAME_SLOTS;i++)if(i!=latest && slots[i].refs==0){selected=i;break;}
      xSemaphoreGive(lock);if(selected<0)continue;
      phone_perf_frame_begin();
      fpga_frame_t frame;bool reuse=false;uint8_t *p=slots[selected].bytes+sizeof(stream_header_t);
      if(!CONFIG_STYLE_MOCK_SOURCE){
        int64_t phase_started=esp_timer_get_time();esp_err_t query_error=fpga_transport_query(&frame);
        phone_perf_frame_phase(PHONE_PERF_QUERY,(uint64_t)(esp_timer_get_time()-phase_started));
        if(query_error!=ESP_OK){phone_perf_frame_end(PHONE_PERF_QUERY_FAILED,query_error,0);continue;}
        // Only this capture task replaces latest. Sender tasks read immutable
        // slots; controls for an unchanged picture get a separate copied slot.
        if(latest>=0){stream_header_t *old=(stream_header_t*)slots[latest].bytes;
          if(frame.frame_id==old->frame_id && (uint16_t)frame.token==(uint16_t)old->style_epoch && frame.style_id==old->style_id){
            if(frame.effect==old->effect && frame.source_fps_x100==old->source_fps_x100){
              phone_perf_frame_end(PHONE_PERF_UNCHANGED,ESP_OK,frame.frame_id);continue;}
            memcpy(p,slots[latest].bytes+sizeof(stream_header_t),STREAM_PAYLOAD_BYTES);
            frame.style_epoch=old->style_epoch;reuse=true;
          }
        }
      }
      if(!reuse){
        int64_t phase_started=esp_timer_get_time();esp_err_t acquire_error=fpga_transport_acquire(&frame);
        phone_perf_frame_phase(PHONE_PERF_ACQUIRE,(uint64_t)(esp_timer_get_time()-phase_started));
        if(acquire_error!=ESP_OK){phone_perf_frame_end(PHONE_PERF_ACQUIRE_FAILED,acquire_error,0);continue;}
        esp_err_t e=ESP_OK;
        phase_started=esp_timer_get_time();
        for(size_t off=0;off<STREAM_PAYLOAD_BYTES && e==ESP_OK;off+=480){size_t n=STREAM_PAYLOAD_BYTES-off;if(n>480)n=480;e=fpga_transport_read_chunk(&frame,off,p+off,n);}
        phone_perf_frame_phase(PHONE_PERF_READ,(uint64_t)(esp_timer_get_time()-phase_started));
        phase_started=esp_timer_get_time();esp_err_t release=fpga_transport_release(&frame);
        phone_perf_frame_phase(PHONE_PERF_RELEASE,(uint64_t)(esp_timer_get_time()-phase_started));
        if(e!=ESP_OK || release!=ESP_OK){phone_perf_frame_end(e!=ESP_OK?PHONE_PERF_READ_FAILED:PHONE_PERF_RELEASE_FAILED,e!=ESP_OK?e:release,frame.frame_id);continue;}
      }
      if(frame.style_id>2 || frame.effect>1){phone_perf_frame_end(PHONE_PERF_INVALID_METADATA,ESP_ERR_INVALID_RESPONSE,frame.frame_id);continue;}
      stream_header_t h={.magic=STREAM_MAGIC,.version=1,.header_bytes=sizeof(h),.frame_id=frame.frame_id,.style_epoch=frame.style_epoch,
        .timestamp_ms=(uint32_t)(esp_timer_get_time()/1000),.style_id=frame.style_id,.effect=frame.effect,.flags=1|(CONFIG_STYLE_MOCK_SOURCE?4:0),
        .width=STREAM_WIDTH,.height=STREAM_HEIGHT,.stride=STREAM_WIDTH*3,.pixel_format=1,.payload_bytes=STREAM_PAYLOAD_BYTES,
        .source_fps_x100=frame.source_fps_x100,.sequence=++sequence};
      memcpy(slots[selected].bytes,&h,sizeof(h));
      xSemaphoreTake(lock,portMAX_DELAY);latest=selected;for(int i=0;i<CLIENTS;i++)if(clients[i].active)xTaskNotifyGive(clients[i].task);xSemaphoreGive(lock);
      phone_perf_frame_end(PHONE_PERF_PUBLISHED,ESP_OK,frame.frame_id);
    }
}
static unsigned active_stream_clients(void){
    unsigned count=0;xSemaphoreTake(lock,portMAX_DELAY);
    for(int i=0;i<CLIENTS;i++)if(clients[i].active)count++;
    xSemaphoreGive(lock);return count;
}
void app_main(void){
    esp_err_t e=nvs_flash_init();if(e==ESP_ERR_NVS_NO_FREE_PAGES || e==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();}ESP_ERROR_CHECK(e);
    lock=xSemaphoreCreateMutex();assert(lock);
    for(int i=0;i<FRAME_SLOTS;i++){slots[i].bytes=heap_caps_malloc(FRAME_BYTES,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);assert(slots[i].bytes);}
    ESP_ERROR_CHECK(esp_netif_init());ESP_ERROR_CHECK(esp_event_loop_create_default());esp_netif_create_default_wifi_ap();
    wifi_init_config_t wc=WIFI_INIT_CONFIG_DEFAULT();ESP_ERROR_CHECK(esp_wifi_init(&wc));
    wifi_config_t ap={.ap={.channel=6,.max_connection=AP_ASSOCIATIONS,.authmode=WIFI_AUTH_WPA2_PSK}};
    strlcpy((char*)ap.ap.ssid,CONFIG_STYLE_AP_SSID,sizeof(ap.ap.ssid));ap.ap.ssid_len=strlen(CONFIG_STYLE_AP_SSID);
    strlcpy((char*)ap.ap.password,CONFIG_STYLE_AP_PASSWORD,sizeof(ap.ap.password));
    if(strlen(CONFIG_STYLE_AP_PASSWORD)<8)ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));ESP_ERROR_CHECK(esp_wifi_start());
    httpd_config_t hc=HTTPD_DEFAULT_CONFIG();hc.max_open_sockets=9;hc.max_uri_handlers=16;hc.stack_size=12288;hc.send_wait_timeout=2;hc.recv_wait_timeout=2;hc.lru_purge_enable=false;hc.close_fn=session_close;
    ESP_ERROR_CHECK(httpd_start(&server,&hc));
    const char *paths[]={"/","/style.css","/app.mjs","/protocol.mjs","/debug","/debug/","/debug/remote_debug.css","/debug/remote_debug.mjs","/debug/remote_api.mjs"};
    for(size_t i=0;i<sizeof(paths)/sizeof(paths[0]);i++){httpd_uri_t u={.uri=paths[i],.method=HTTP_GET,.handler=asset};ESP_ERROR_CHECK(httpd_register_uri_handler(server,&u));}
    httpd_uri_t ws={.uri="/stream",.method=HTTP_GET,.handler=stream,.is_websocket=true,.handle_ws_control_frames=true,.ws_post_handshake_cb=stream_open};ESP_ERROR_CHECK(httpd_register_uri_handler(server,&ws));
    e=fpga_transport_init();if(e==ESP_OK){microphone_start();assert(xTaskCreate(capture,"capture",4096,NULL,4,NULL)==pdPASS);}
    else ESP_LOGE(TAG,"Physical FPGA transport unavailable; no video or microphone task started");
    ESP_ERROR_CHECK(remote_debug_start(server,e==ESP_OK,active_stream_clients));
    httpd_uri_t perf={.uri="/api/perf",.method=HTTP_GET,.handler=perf_handler};ESP_ERROR_CHECK(httpd_register_uri_handler(server,&perf));
    ESP_LOGI(TAG,"Connect to %s, then http://192.168.4.1; %d FPS shared capture, max %d clients",CONFIG_STYLE_AP_SSID,CAPTURE_FPS,CLIENTS);
    ESP_LOGI(TAG,"Computer controls: http://192.168.4.1/debug; %d hotspot associations, %d independent video clients",AP_ASSOCIATIONS,CLIENTS);
}
