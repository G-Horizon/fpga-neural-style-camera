#include "remote_debug.h"
#include "remote_control.h"
#include "remote_json.h"
#include "fpga_transport.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define POLL_MS 250
#define CONNECTED_MAX_AGE_MS 2000
#define RECEIVE_QUEUE_DEPTH 2
#define JSON_CAPACITY 12288
static const char *TAG="remote_debug";
static remote_control_t control;
static SemaphoreHandle_t state_lock;
static QueueHandle_t receive_queue;
static TaskHandle_t worker_task;
static bool driver_ready, have_response, extended_capability, have_live;
static int64_t last_response_ms, polled_at_ms;
static esp_err_t last_error=ESP_OK;
static bool frame_valid;
static uint32_t latest_frame_id;
static fpga_live_status_t live;
static remote_client_count_fn stream_count;
static int64_t now_ms(void) { return esp_timer_get_time()/1000; }
static bool connected_at(int64_t now) { return driver_ready && have_response && now-last_response_ms<=CONNECTED_MAX_AGE_MS; }

static void nullable_bool(remote_json_t *parent,const char *key,bool known,bool value) { if(known)remote_json_bool(parent,key,value);else remote_json_null(parent,key); }
static void nullable_number(remote_json_t *parent,const char *key,bool known,int64_t value) { if(known)remote_json_number(parent,key,value);else remote_json_null(parent,key); }
static void record_json(remote_json_t *j,const remote_record_t *r,bool version) {
    remote_json_begin(j,NULL,'{');
    if(version)remote_json_number(j,"version",1);
    remote_json_string(j,"request_id",r->request.request_id);
    remote_json_string(j,"command",remote_command_name(r->request.command));
    if(!remote_has_value(r->request.command))remote_json_null(j,"value");
    else if(r->request.command==REMOTE_FPS)remote_json_bool(j,"value",r->request.value);
    else remote_json_number(j,"value",r->request.value);
    remote_json_string(j,"state",remote_state_name(r->state));
    remote_json_string(j,"message",r->message);
    if(r->error[0])remote_json_string(j,"error",r->error);else remote_json_null(j,"error");
    remote_json_number(j,"created_at_ms",r->created_at_ms);
    nullable_number(j,"completed_at_ms",r->state!=REMOTE_QUEUED,r->completed_at_ms);
    remote_json_end(j,'}');
}
static esp_err_t send_json(httpd_req_t *req,int status,remote_json_t *json) {
    const char *line=status==200?"200 OK":status==202?"202 Accepted":status==400?"400 Bad Request":status==409?"409 Conflict":status==422?"422 Unprocessable Content":"503 Service Unavailable";
    char *body=json&&!json->failed?json->buffer:NULL;
    httpd_resp_set_status(req,body?line:"503 Service Unavailable");
    httpd_resp_set_type(req,"application/json; charset=utf-8");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    httpd_resp_set_hdr(req,"Connection","close");
    esp_err_t e=httpd_resp_send(req,body?body:"{\"error\":\"out_of_memory\"}",HTTPD_RESP_USE_STRLEN);
    if(json)free(json->buffer);
    return e;
}
static esp_err_t error_json(httpd_req_t *req,int status,const char *error) {
    remote_json_t j;remote_json_init(&j,malloc(256),256);remote_json_begin(&j,NULL,'{');remote_json_string(&j,"error",error);remote_json_end(&j,'}');return send_json(req,status,&j);
}
static esp_err_t status_handler(httpd_req_t *req) {
    int64_t now=now_ms(); wifi_sta_list_t stations={0}; unsigned ap_clients=0;
    if(esp_wifi_ap_get_sta_list(&stations)==ESP_OK)ap_clients=stations.num;
    unsigned viewers=stream_count?stream_count():0;
    // Snapshot the small cache, then release the lock before JSON/HTTP work.
    remote_record_t records[REMOTE_RECORDS]; fpga_live_status_t snapshot; bool connected,ext,known,frames;uint32_t fid;int64_t polled;esp_err_t error;
    xSemaphoreTake(state_lock,portMAX_DELAY);
    memcpy(records,control.records,sizeof(records));snapshot=live;connected=connected_at(now);ext=extended_capability;known=connected&&ext&&have_live;
    frames=frame_valid;fid=latest_frame_id;polled=polled_at_ms;error=last_error;
    xSemaphoreGive(state_lock);
    remote_json_t json;remote_json_init(&json,malloc(JSON_CAPACITY),JSON_CAPACITY);remote_json_t *j=&json;
    remote_json_begin(j,NULL,'{');remote_json_number(j,"version",1);remote_json_string(j,"variant","remote_debug");remote_json_bool(j,"mock_source",CONFIG_STYLE_MOCK_SOURCE);
    remote_json_begin(j,"bridge",'{');remote_json_bool(j,"transport_ready",driver_ready);remote_json_bool(j,"connected",connected);
    if(error==ESP_OK)remote_json_null(j,"last_error");else remote_json_string(j,"last_error",esp_err_to_name(error));
    nullable_number(j,"polled_at_ms",polled>0,polled);nullable_number(j,"status_age_ms",polled>0,now-polled);
    remote_json_number(j,"ap_clients",ap_clients);remote_json_number(j,"stream_clients",viewers);remote_json_number(j,"stream_capacity",3);
    remote_json_number(j,"uptime_ms",now);remote_json_number(j,"free_heap_bytes",esp_get_free_heap_size());remote_json_end(j,'}');
    remote_json_begin(j,"capabilities",'{');remote_json_bool(j,"style",connected);remote_json_bool(j,"extended",connected&&ext);remote_json_end(j,'}');
    remote_json_begin(j,"fpga",'{');
    nullable_number(j,"active_style",known&&snapshot.active_style>=1&&snapshot.active_style<=3,snapshot.active_style);
    nullable_number(j,"target_style",known&&snapshot.target_style>=1&&snapshot.target_style<=3,snapshot.target_style);
    nullable_bool(j,"target_pending",known,snapshot.target_pending);nullable_bool(j,"model_valid",known,snapshot.model_valid);
    nullable_number(j,"active_epoch",known,snapshot.active_epoch);
    nullable_number(j,"requested_mode",known,snapshot.requested_mode);nullable_number(j,"accepted_mode",known,snapshot.accepted_mode);nullable_number(j,"actual_mode",known,snapshot.actual_mode);
    nullable_bool(j,"fps_requested",known,snapshot.fps_requested);nullable_bool(j,"fps_actual",known,snapshot.fps_actual);
    nullable_bool(j,"effect_requested",known,snapshot.effect_requested);nullable_bool(j,"effect_actual",known,snapshot.effect_actual);
    nullable_number(j,"source_fps",known,snapshot.source_fps);nullable_bool(j,"frame_valid",connected,frames);nullable_number(j,"latest_frame_id",connected&&frames,fid);remote_json_end(j,'}');
    remote_json_begin(j,"commands",'[');
    // Order records oldest to newest without changing the shared record slots.
    for(int n=0;n<REMOTE_RECORDS;n++) { int index=-1;uint32_t order=UINT32_MAX;for(int i=0;i<REMOTE_RECORDS;i++)if(records[i].used && records[i].order<order){index=i;order=records[i].order;}if(index<0)break;record_json(j,&records[index],false);records[index].used=false; }
    remote_json_end(j,']');remote_json_end(j,'}');return send_json(req,200,j);
}
static bool json_content_type(httpd_req_t *req) {
    char type[80];size_t n=httpd_req_get_hdr_value_len(req,"Content-Type");
    if(!n || n>=sizeof(type) || httpd_req_get_hdr_value_str(req,"Content-Type",type,sizeof(type))!=ESP_OK)return false;
    char *semicolon=strchr(type,';');if(semicolon)*semicolon=0;
    char *end=type+strlen(type);while(end>type && end[-1]==' ')*--end=0;
    return !strcasecmp(type,"application/json");
}
static void receive_control(httpd_req_t *req) {
    if(!json_content_type(req)) { error_json(req,400,"content_type_must_be_application_json");return; }
    if(req->content_len==0 || req->content_len>REMOTE_BODY_MAX) { error_json(req,400,"body_size_must_be_1_to_192_bytes");return; }
    char body[REMOTE_BODY_MAX+1];size_t received=0;
    // This receive runs in a dedicated task, never the HTTPD video ACK task.
    while(received<req->content_len) { int n=httpd_req_recv(req,body+received,req->content_len-received);if(n<=0){error_json(req,400,n==HTTPD_SOCK_ERR_TIMEOUT?"request_body_timeout":"incomplete_request_body");return;}received+=(size_t)n; }
    body[received]=0;remote_request_t request;const char *error;
    if(!remote_parse(body,received,&request,&error)) { error_json(req,400,error);return; }
    // Reserve the complete reply before accepting anything. An allocation
    // failure must mean no command was submitted, rather than a 503 after
    // the worker may already have acted on the request.
    char *reply_buffer=malloc(1024);
    if(!reply_buffer){error_json(req,503,"out_of_memory");return;}
    int index;remote_record_t record={0};xSemaphoreTake(state_lock,portMAX_DELAY);
    int status=remote_submit(&control,&request,connected_at(now_ms()),extended_capability,now_ms(),&index,&error);
    if(index>=0)record=control.records[index];
    xSemaphoreGive(state_lock);
    if(status>=400){free(reply_buffer);error_json(req,status,error);return;}
    if(status==202)xTaskNotifyGive(worker_task);
    remote_json_t j;remote_json_init(&j,reply_buffer,1024);record_json(&j,&record,true);send_json(req,status,&j);
}
static void receive_task(void *unused) {
    (void)unused;for(;;){httpd_req_t *req=NULL;if(xQueueReceive(receive_queue,&req,portMAX_DELAY)==pdTRUE){receive_control(req);httpd_req_async_handler_complete(req);}}
}
static esp_err_t control_handler(httpd_req_t *req) {
    httpd_req_t *async=NULL;esp_err_t e=httpd_req_async_handler_begin(req,&async);if(e!=ESP_OK)return error_json(req,503,"http_receiver_busy");
    if(xQueueSend(receive_queue,&async,0)!=pdTRUE){error_json(async,503,"http_receiver_busy");httpd_req_async_handler_complete(async);}return ESP_OK;
}
static void poll_fpga(void) {
    bool ext=false,frames=false;uint32_t fid=0;fpga_live_status_t snapshot={0};
    esp_err_t e=fpga_transport_probe(&ext,&frames,&fid);bool probe_ok=e==ESP_OK;
    if(probe_ok && ext)e=fpga_transport_live(&snapshot);
    int64_t now=now_ms();xSemaphoreTake(state_lock,portMAX_DELAY);
    last_error=e;
    // A valid probe establishes connectivity even if the live query fails.
    if(probe_ok){have_response=true;last_response_ms=now;extended_capability=ext;frame_valid=frames;latest_frame_id=fid;}
    if(e==ESP_OK){polled_at_ms=now;have_live=ext;if(ext){live=snapshot;frame_valid=snapshot.frame_valid;latest_frame_id=snapshot.latest_frame_id;}}
    else have_live=false;
    xSemaphoreGive(state_lock);
}
static void control_worker(void *unused) {
    (void)unused;int64_t next_poll=0;
    for(;;){
        if(driver_ready && now_ms()>=next_poll){poll_fpga();next_poll=now_ms()+POLL_MS;}
        remote_request_t request={0};bool connected,ext;int index;
        xSemaphoreTake(state_lock,portMAX_DELAY);index=remote_dequeue(&control);if(index>=0)request=control.records[index].request;connected=connected_at(now_ms());ext=extended_capability;xSemaphoreGive(state_lock);
        if(index>=0){esp_err_t e=ESP_ERR_INVALID_STATE;bool uncertain=false;
            if(connected){
                if(request.command==REMOTE_STYLE || request.command==REMOTE_NEXT)e=fpga_transport_remote_style((uint8_t)request.value,request.command==REMOTE_NEXT,&uncertain);
                else if(ext){uint8_t action=request.command==REMOTE_FPS?1:request.command==REMOTE_MODE?2:request.command==REMOTE_MODE_NEXT?3:4;e=fpga_transport_remote(action,(uint8_t)request.value,&uncertain);}
                else e=ESP_ERR_NOT_SUPPORTED;
            }
            remote_state_t state=e==ESP_OK?REMOTE_SENT:uncertain?REMOTE_UNCERTAIN:REMOTE_FAILED;
            const char *message=e==ESP_OK?"FPGA acknowledged receipt; verify actual state in subsequent status queries":uncertain?"Outcome unknown; inspect current state before deciding whether to issue a new request":"Command was not accepted; no automatic retry";
            xSemaphoreTake(state_lock,portMAX_DELAY);remote_complete(&control,index,state,now_ms(),e==ESP_OK?NULL:esp_err_to_name(e),message);if(e==ESP_OK){have_response=true;last_response_ms=now_ms();}xSemaphoreGive(state_lock);
            // Query after a command, including uncertain outcomes; never resend it.
            next_poll=0;continue;
        }
        ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(POLL_MS));
    }
}
esp_err_t remote_debug_start(httpd_handle_t server,bool transport_ready,remote_client_count_fn clients) {
    driver_ready=transport_ready;stream_count=clients;remote_control_init(&control);
    state_lock=xSemaphoreCreateMutex();receive_queue=xQueueCreate(RECEIVE_QUEUE_DEPTH,sizeof(httpd_req_t*));
    if(!state_lock || !receive_queue)return ESP_ERR_NO_MEM;
    // Status JSON uses a bounded 16-record stack snapshot. Keep the HTTPD task
    // stack at 12 KiB in main.c; worker/receiver do not run on that task.
    if(xTaskCreate(control_worker,"remote_spi",4096,NULL,3,&worker_task)!=pdPASS || xTaskCreate(receive_task,"remote_http",4096,NULL,2,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
    httpd_uri_t status={.uri="/api/status",.method=HTTP_GET,.handler=status_handler};
    httpd_uri_t post={.uri="/api/control",.method=HTTP_POST,.handler=control_handler};
    esp_err_t e=httpd_register_uri_handler(server,&status);if(e==ESP_OK)e=httpd_register_uri_handler(server,&post);
    ESP_LOGI(TAG,"Remote debug API ready; driver=%d mock=%d (FPGA connectivity is probed independently)",driver_ready,CONFIG_STYLE_MOCK_SOURCE);return e;
}
