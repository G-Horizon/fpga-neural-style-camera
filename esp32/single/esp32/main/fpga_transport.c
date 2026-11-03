#include "fpga_transport.h"
#include "phone_perf.h"
#include <stdio.h>
#include "esp_private/esp_clk.h"
#include <string.h>
#include <assert.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_rom_crc.h"
#include "esp_log.h"
#define SPI_MAGIC UINT32_C(0x32535046)
#define PACKET 512
#define HEADER 32
#define DATA 480
typedef struct __attribute__((packed)) {
    uint32_t magic;uint8_t version,opcode,flags,status;
    uint32_t seq,frame_id,offset;uint16_t length;uint8_t style,fps;
    uint16_t epoch;uint8_t effect,reserved;uint32_t crc;
} spi_header_t;
_Static_assert(sizeof(spi_header_t)==32,"SPI FPS2 layout");
static uint32_t mock_id,request_seq,audio_index;
static uint32_t expanded_epoch;static uint16_t last_epoch;static bool have_epoch;
static spi_device_handle_t device;static SemaphoreHandle_t bus_lock;
static uint8_t *tx,*rx;
static portMUX_TYPE mock_lock=portMUX_INITIALIZER_UNLOCKED;
static fpga_live_status_t mock_live={.extended=true,.frame_valid=true,.model_valid=true,.active_style=1,.target_style=1,.active_epoch=1,.fps_requested=true,.fps_actual=true};
static int64_t mock_effect_until;
static bool crc_selftest_pass;
static uint32_t crc_selftest_vectors;
// This instrumentation is passive. It never controls GPIO, SPI, lease, or retry.
static portMUX_TYPE perf_lock=portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t perf_owner;
static int64_t perf_started;
static phone_frame_perf_t perf_live,perf_last,perf_sum;
static phone_spi_perf_t perf_all;
static uint32_t perf_attempts,perf_published,perf_unchanged,perf_failed;
static void perf_add_spi(phone_spi_perf_t *a,const phone_spi_perf_t *b){
    a->cmd_us+=b->cmd_us;a->ready_us+=b->ready_us;a->nop_us+=b->nop_us;
    a->crc_us+=b->crc_us;a->bus_lock_us+=b->bus_lock_us;
    a->command_total_us+=b->command_total_us;a->delay_us+=b->delay_us;a->copy_us+=b->copy_us;a->driver_bus_us+=b->driver_bus_us;
    a->commands+=b->commands;a->command_failures+=b->command_failures;
    a->ready_sleep_count+=b->ready_sleep_count;a->ready_slept_commands+=b->ready_slept_commands;
    if(b->ready_max_us>a->ready_max_us)a->ready_max_us=b->ready_max_us;
}
static void perf_command(const phone_spi_perf_t *sample){
    TaskHandle_t caller=xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&perf_lock);perf_add_spi(&perf_all,sample);
    if(perf_owner==caller)perf_add_spi(&perf_live.spi,sample);
    portEXIT_CRITICAL(&perf_lock);
}
void phone_perf_frame_begin(void){
    int64_t now=esp_timer_get_time();TaskHandle_t caller=xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&perf_lock);memset(&perf_live,0,sizeof(perf_live));
    perf_owner=caller;perf_started=now;perf_attempts++;portEXIT_CRITICAL(&perf_lock);
}
void phone_perf_frame_phase(unsigned phase,uint64_t elapsed_us){
    TaskHandle_t caller=xTaskGetCurrentTaskHandle();portENTER_CRITICAL(&perf_lock);
    if(perf_owner==caller){
        if(phase==PHONE_PERF_QUERY)perf_live.query_us+=elapsed_us;
        else if(phase==PHONE_PERF_ACQUIRE)perf_live.acquire_us+=elapsed_us;
        else if(phase==PHONE_PERF_READ)perf_live.read_us+=elapsed_us;
        else if(phase==PHONE_PERF_RELEASE)perf_live.release_us+=elapsed_us;
    }portEXIT_CRITICAL(&perf_lock);
}
void phone_perf_frame_end(unsigned result,esp_err_t error,uint32_t frame_id){
    int64_t now=esp_timer_get_time();TaskHandle_t caller=xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&perf_lock);
    if(perf_owner==caller){
        perf_live.total_us=(uint64_t)(now-perf_started);perf_live.result=result;
        perf_live.error=error;perf_live.frame_id=frame_id;perf_last=perf_live;
        perf_sum.total_us+=perf_live.total_us;perf_sum.query_us+=perf_live.query_us;
        perf_sum.acquire_us+=perf_live.acquire_us;perf_sum.read_us+=perf_live.read_us;
        perf_sum.release_us+=perf_live.release_us;perf_add_spi(&perf_sum.spi,&perf_live.spi);
        if(result==PHONE_PERF_PUBLISHED)perf_published++;
        else if(result==PHONE_PERF_UNCHANGED)perf_unchanged++;else perf_failed++;
        perf_owner=NULL;
    }portEXIT_CRITICAL(&perf_lock);
}
static int perf_spi_json(char *b,size_t n,const phone_spi_perf_t *s){
    return snprintf(b,n,"{\"commands\":%lu,\"command_failures\":%lu,\"cmd_us\":%llu,\"ready_us\":%llu,"
        "\"ready_sleep_count\":%lu,\"ready_slept_commands\":%lu,\"ready_max_us\":%lu,\"nop_us\":%llu,"
        "\"crc_us\":%llu,\"bus_lock_us\":%llu,\"command_total_us\":%llu,\"delay_us\":%llu,\"copy_us\":%llu,\"driver_bus_us\":%llu}",
        (unsigned long)s->commands,(unsigned long)s->command_failures,
        (unsigned long long)s->cmd_us,(unsigned long long)s->ready_us,
        (unsigned long)s->ready_sleep_count,(unsigned long)s->ready_slept_commands,
        (unsigned long)s->ready_max_us,(unsigned long long)s->nop_us,(unsigned long long)s->crc_us,
        (unsigned long long)s->bus_lock_us,(unsigned long long)s->command_total_us,
        (unsigned long long)s->delay_us,(unsigned long long)s->copy_us,(unsigned long long)s->driver_bus_us);
}
static int perf_frame_json(char *b,size_t n,const phone_frame_perf_t *s){
    int k=snprintf(b,n,"{\"total_us\":%llu,\"query_us\":%llu,\"acquire_us\":%llu,\"read_us\":%llu,\"release_us\":%llu,"
        "\"frame_id\":%lu,\"result\":%lu,\"error\":%ld,\"spi\":",
        (unsigned long long)s->total_us,(unsigned long long)s->query_us,(unsigned long long)s->acquire_us,
        (unsigned long long)s->read_us,(unsigned long long)s->release_us,(unsigned long)s->frame_id,
        (unsigned long)s->result,(long)s->error);
    if(k<0||(size_t)k>=n)return -1;
    int j=perf_spi_json(b+k,n-k,&s->spi);if(j<0||(size_t)j>=n-k)return -1;k+=j;
    if((size_t)k+1>=n)return -1;
    b[k++]='}';b[k]=0;return k;
}
int phone_perf_json(char *buffer,size_t capacity){
    phone_frame_perf_t last,sum;phone_spi_perf_t all;
    uint32_t attempts,published,unchanged,failed;bool active;
    portENTER_CRITICAL(&perf_lock);last=perf_last;sum=perf_sum;all=perf_all;
    attempts=perf_attempts;published=perf_published;unchanged=perf_unchanged;failed=perf_failed;
    active=perf_owner!=NULL;portEXIT_CRITICAL(&perf_lock);
    int k=snprintf(buffer,capacity,"{\"version\":1,\"cpu_mhz\":%d,\"configured_cpu_mhz\":%d,\"tick_hz\":%d,"
        "\"spi_mhz\":%d,\"target_fps\":%d,\"crc_backend\":\"esp_rom_crc32_le\",\"crc_selftest_pass\":%s,\"crc_selftest_vectors\":%lu,\"spi_explicit_acquire\":true,\"frame_active\":%s,\"attempts\":%lu,\"published\":%lu,"
        "\"unchanged\":%lu,\"failed\":%lu,\"last\":",esp_clk_cpu_freq()/1000000,CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        CONFIG_FREERTOS_HZ,CONFIG_STYLE_SPI_CLOCK_MHZ,CONFIG_STYLE_PHYSICAL_FPS,crc_selftest_pass?"true":"false",(unsigned long)crc_selftest_vectors,active?"true":"false",
        (unsigned long)attempts,(unsigned long)published,(unsigned long)unchanged,(unsigned long)failed);
    if(k<0||(size_t)k>=capacity)return -1;
    int j=perf_frame_json(buffer+k,capacity-k,&last);if(j<0||(size_t)j>=capacity-k)return -1;k+=j;
    j=snprintf(buffer+k,capacity-k,",\"sum\":");if(j<0||(size_t)j>=capacity-k)return -1;k+=j;
    j=perf_frame_json(buffer+k,capacity-k,&sum);if(j<0||(size_t)j>=capacity-k)return -1;k+=j;
    j=snprintf(buffer+k,capacity-k,",\"all_spi\":");if(j<0||(size_t)j>=capacity-k)return -1;k+=j;
    j=perf_spi_json(buffer+k,capacity-k,&all);if(j<0||(size_t)j>=capacity-k)return -1;k+=j;
    if((size_t)k+1>=capacity)return -1;
    buffer[k++]='}';buffer[k]=0;return k;
}
// Retained only for one startup self-test; all transaction CRC uses the ROM API.
static uint32_t crc_reference(const uint8_t *p,size_t n){uint32_t c=UINT32_MAX;for(size_t i=0;i<n;i++){c^=p[i];for(int j=0;j<8;j++)c=(c>>1)^((c&1)?UINT32_C(0xedb88320):0);}return c^UINT32_MAX;}
static uint32_t crc32(const uint8_t *p,size_t n){return esp_rom_crc32_le(0,p,(uint32_t)n);}
static bool crc_startup_selftest(void){
    uint8_t data[480];uint32_t random=UINT32_C(0x6d2b79f5);const size_t lengths[]={0,192,480};
    crc_selftest_vectors=0;
    if(crc32(NULL,0)!=0)return false;
    crc_selftest_vectors++;
    if(crc32((const uint8_t*)"123456789",9)!=UINT32_C(0xcbf43926))return false;
    crc_selftest_vectors++;
    for(unsigned round=0;round<32;round++){
        for(size_t i=0;i<sizeof(data);i++){random^=random<<13;random^=random>>17;random^=random<<5;data[i]=(uint8_t)random;}
        for(unsigned i=0;i<sizeof(lengths)/sizeof(lengths[0]);i++){
            if(crc32(data,lengths[i])!=crc_reference(data,lengths[i]))return false;
            crc_selftest_vectors++;
        }
    }return true;
}
static esp_err_t ready(phone_spi_perf_t *sample){
    int64_t end=esp_timer_get_time()+100000;unsigned spins=0;
    while(!gpio_get_level(4)){if(esp_timer_get_time()>end)return ESP_ERR_TIMEOUT;
      if(++spins<40)esp_rom_delay_us(2);else {sample->ready_sleep_count++;vTaskDelay(1);}}
    return ESP_OK;
}
static esp_err_t transfer(void){spi_transaction_t t={.length=PACKET*8,.tx_buffer=tx,.rx_buffer=rx};return spi_device_polling_transmit(device,&t);}
// Serialized CMD + READY wait + NOP. Release the bus between video chunks so
// 20 ms microphone blocks can interleave without dropping the video lease.
static esp_err_t command_ex(spi_header_t *h,const uint8_t *in,uint8_t *out,spi_header_t *reply,bool *uncertain){
    phone_spi_perf_t sample={.commands=1};int64_t began=esp_timer_get_time(),t=began;
    if(uncertain)*uncertain=false;
    bool locked=xSemaphoreTake(bus_lock,pdMS_TO_TICKS(200))==pdTRUE;
    sample.bus_lock_us=(uint64_t)(esp_timer_get_time()-t);
    if(!locked){sample.command_failures=1;sample.command_total_us=(uint64_t)(esp_timer_get_time()-began);perf_command(&sample);return ESP_ERR_TIMEOUT;}
    t=esp_timer_get_time();esp_err_t acquire_error=spi_device_acquire_bus(device,portMAX_DELAY);
    sample.driver_bus_us=(uint64_t)(esp_timer_get_time()-t);
    if(acquire_error!=ESP_OK){xSemaphoreGive(bus_lock);sample.command_failures=1;
      sample.command_total_us=(uint64_t)(esp_timer_get_time()-began);perf_command(&sample);return acquire_error;}
    h->magic=SPI_MAGIC;h->version=1;h->seq=++request_seq;
    t=esp_timer_get_time();h->crc=crc32(in,in?h->length:0);sample.crc_us+=(uint64_t)(esp_timer_get_time()-t);
    memset(tx,0,PACKET);memcpy(tx,h,HEADER);if(in)memcpy(tx+HEADER,in,h->length);
    // READY starts low at FPGA reset: the first CMD must not wait for a
    // nonexistent previous response. Every successful command consumes NOP.
    if(uncertain)*uncertain=true;
    t=esp_timer_get_time();esp_err_t e=transfer();sample.cmd_us=(uint64_t)(esp_timer_get_time()-t);
    if(e==ESP_OK){
      t=esp_timer_get_time();esp_rom_delay_us(5);sample.delay_us=(uint64_t)(esp_timer_get_time()-t);
      t=esp_timer_get_time();e=ready(&sample);sample.ready_us=(uint64_t)(esp_timer_get_time()-t);
      sample.ready_max_us=(uint32_t)sample.ready_us;sample.ready_slept_commands=sample.ready_sleep_count!=0;
    }
    if(e==ESP_OK){memset(tx,0,PACKET);spi_header_t nop={.magic=SPI_MAGIC,.version=1};memcpy(tx,&nop,HEADER);
      t=esp_timer_get_time();e=transfer();sample.nop_us=(uint64_t)(esp_timer_get_time()-t);}
    if(e==ESP_OK){spi_header_t r;memcpy(&r,rx,HEADER);
      if(r.magic!=SPI_MAGIC || r.version!=1 || r.opcode!=h->opcode || r.seq!=h->seq || r.length>DATA)e=ESP_ERR_INVALID_RESPONSE;
      else {
        t=esp_timer_get_time();uint32_t received_crc=crc32(rx+HEADER,r.length);sample.crc_us+=(uint64_t)(esp_timer_get_time()-t);
        if(r.crc!=received_crc)e=ESP_ERR_INVALID_CRC;
        else {
          if(reply)*reply=r;
          // A valid FPGA rejection is definitive, unlike a lost/corrupt ACK.
          if(uncertain)*uncertain=false;
          if(r.status)e=r.status==1?ESP_ERR_NOT_FOUND:r.status==2?ESP_ERR_INVALID_STATE:ESP_FAIL;
          else if(out && r.length!=h->length){e=ESP_ERR_INVALID_RESPONSE;if(uncertain)*uncertain=true;}
          else if(out && r.length){t=esp_timer_get_time();memcpy(out,rx+HEADER,r.length);sample.copy_us=(uint64_t)(esp_timer_get_time()-t);}
        }
      }
    }
    spi_device_release_bus(device);
    xSemaphoreGive(bus_lock);sample.command_failures=e!=ESP_OK;
    sample.command_total_us=(uint64_t)(esp_timer_get_time()-began);perf_command(&sample);return e;
}
static esp_err_t command(spi_header_t *h,const uint8_t *in,uint8_t *out,spi_header_t *reply){return command_ex(h,in,out,reply,NULL);}
esp_err_t fpga_transport_init(void){
    if(CONFIG_STYLE_MOCK_SOURCE){ESP_LOGW("transport","MOCK frames; no physical FPGA video/audio transactions");return ESP_OK;}
    crc_selftest_pass=crc_startup_selftest();
    if(!crc_selftest_pass)return ESP_ERR_INVALID_CRC;
    bus_lock=xSemaphoreCreateMutex();assert(bus_lock);
    tx=heap_caps_calloc(1,PACKET,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);rx=heap_caps_calloc(1,PACKET,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);assert(tx && rx);
    gpio_config_t g={.pin_bit_mask=UINT64_C(1)<<4,.mode=GPIO_MODE_INPUT};ESP_ERROR_CHECK(gpio_config(&g));
    spi_bus_config_t b={.mosi_io_num=11,.miso_io_num=13,.sclk_io_num=12,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=PACKET};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&b,SPI_DMA_CH_AUTO));
    spi_device_interface_config_t d={.clock_speed_hz=CONFIG_STYLE_SPI_CLOCK_MHZ*1000000,.mode=0,.spics_io_num=10,.queue_size=1,.cs_ena_pretrans=1,.cs_ena_posttrans=1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST,&d,&device));return ESP_OK;
}
esp_err_t fpga_transport_status(void){if(CONFIG_STYLE_MOCK_SOURCE)return ESP_OK;spi_header_t h={.opcode=1};return command(&h,NULL,NULL,NULL);}
esp_err_t fpga_transport_query(fpga_frame_t *f){
    if(CONFIG_STYLE_MOCK_SOURCE){portENTER_CRITICAL(&mock_lock);*f=(fpga_frame_t){.frame_id=mock_id,.style_epoch=mock_live.active_epoch,.style_id=mock_live.active_style-1,.source_fps_x100=CONFIG_STYLE_STREAM_FPS*100,.effect=mock_live.effect_requested,.token=mock_live.active_epoch};portEXIT_CRITICAL(&mock_lock);return ESP_OK;}
    spi_header_t h={.opcode=1},r;esp_err_t e=command(&h,NULL,NULL,&r);if(e!=ESP_OK)return e;
    if(!(r.flags&1))return ESP_ERR_NOT_FOUND;
    if(r.style<1 || r.style>3 || r.effect>1 || r.offset!=STREAM_PAYLOAD_BYTES)return ESP_ERR_INVALID_RESPONSE;
    *f=(fpga_frame_t){.frame_id=r.frame_id,.style_epoch=r.epoch,.source_fps_x100=r.fps*100,
        .style_id=r.style-1,.effect=r.effect,.token=r.epoch};return ESP_OK;
}
esp_err_t fpga_transport_acquire(fpga_frame_t *f){
    if(CONFIG_STYLE_MOCK_SOURCE){portENTER_CRITICAL(&mock_lock);uint32_t id=++mock_id;bool effect=esp_timer_get_time()<mock_effect_until;mock_live.effect_requested=mock_live.effect_actual=effect;mock_live.latest_frame_id=id;*f=(fpga_frame_t){.frame_id=id,.style_epoch=mock_live.active_epoch,.style_id=mock_live.active_style-1,.source_fps_x100=CONFIG_STYLE_STREAM_FPS*100,.effect=effect,.token=id};portEXIT_CRITICAL(&mock_lock);return ESP_OK;}
    spi_header_t h={.opcode=2},r;esp_err_t e=command(&h,NULL,NULL,&r);if(e!=ESP_OK)return e;
    if(r.style<1 || r.style>3 || r.effect>1 || r.offset!=STREAM_PAYLOAD_BYTES){fpga_frame_t bad={.frame_id=r.frame_id};fpga_transport_release(&bad);return ESP_ERR_INVALID_RESPONSE;}
    if(!have_epoch){expanded_epoch=r.epoch;have_epoch=true;}else expanded_epoch+=(uint16_t)(r.epoch-last_epoch);last_epoch=r.epoch;
    *f=(fpga_frame_t){.frame_id=r.frame_id,.style_epoch=expanded_epoch,.source_fps_x100=r.fps*100,.style_id=r.style-1,.effect=r.effect,.token=r.epoch};return ESP_OK;
}
esp_err_t fpga_transport_read_chunk(const fpga_frame_t *f,size_t offset,uint8_t *out,size_t n){
    if(offset>STREAM_PAYLOAD_BYTES || n>STREAM_PAYLOAD_BYTES-offset || offset%3 || n%3)return ESP_ERR_INVALID_SIZE;
    if(CONFIG_STYLE_MOCK_SOURCE){for(size_t j=0;j<n;j++){size_t p=(offset+j)/3;unsigned c=(offset+j)%3;out[j]=c==0?(p%256):c==1?(p/256)*255/143:(uint8_t)(f->style_id*80+f->frame_id*3);}return ESP_OK;}
    while(n){uint16_t bytes=n>DATA?DATA:n;spi_header_t h={.opcode=3,.frame_id=f->frame_id,.offset=offset,.length=bytes},r;
      esp_err_t e=command(&h,NULL,out,&r);if(e!=ESP_OK)return e;
      if(r.frame_id!=f->frame_id || r.offset!=offset || r.length!=bytes || r.epoch!=(uint16_t)f->token || r.style!=f->style_id+1 || r.effect!=f->effect)return ESP_ERR_INVALID_RESPONSE;
      out+=bytes;offset+=bytes;n-=bytes;
    }return ESP_OK;
}
esp_err_t fpga_transport_release(const fpga_frame_t *f){if(CONFIG_STYLE_MOCK_SOURCE)return ESP_OK;spi_header_t h={.opcode=4,.frame_id=f->frame_id};return command(&h,NULL,NULL,NULL);}
esp_err_t fpga_transport_audio(const int16_t *s,size_t n,uint32_t sequence){
    (void)sequence;if(CONFIG_STYLE_MOCK_SOURCE)return ESP_OK;
    esp_err_t result=ESP_OK;
    while(n){size_t samples=n>DATA/2?DATA/2:n;spi_header_t h={.opcode=5,.offset=audio_index,.length=samples*2};
      esp_err_t e=command(&h,(const uint8_t*)s,NULL,NULL);audio_index+=samples;
      if(e!=ESP_OK)result=e;
      s+=samples;n-=samples;
    }return result;
}
// Only the microphone task owns this sample counter. A DMA overrun must not
// silently concatenate non-contiguous audio into a recognizer window.
void fpga_transport_audio_discontinuity(uint32_t skipped_samples){audio_index+=skipped_samples;}

esp_err_t fpga_transport_set_style(uint8_t style_id,bool next){
    return fpga_transport_remote_style(style_id,next,NULL);
}
esp_err_t fpga_transport_remote_style(uint8_t style_id,bool next,bool *uncertain){
    if(uncertain)*uncertain=false;
    if(!next && (style_id<1 || style_id>3))return ESP_ERR_INVALID_ARG;
    if(CONFIG_STYLE_MOCK_SOURCE){portENTER_CRITICAL(&mock_lock);mock_live.active_style=next?mock_live.active_style%3+1:style_id;mock_live.target_style=mock_live.active_style;mock_live.active_epoch++;portEXIT_CRITICAL(&mock_lock);return ESP_OK;}
    spi_header_t h={.opcode=6,.flags=next?1:0,.style=next?0:style_id};
    return command_ex(&h,NULL,NULL,NULL,uncertain);
}
esp_err_t fpga_transport_probe(bool *extended,bool *frame_valid,uint32_t *latest_id){
    if(CONFIG_STYLE_MOCK_SOURCE){portENTER_CRITICAL(&mock_lock);*extended=true;*frame_valid=mock_live.frame_valid;*latest_id=mock_live.latest_frame_id;portEXIT_CRITICAL(&mock_lock);return ESP_OK;}
    spi_header_t h={.opcode=1},r={0};esp_err_t e=command(&h,NULL,NULL,&r);
    if(e!=ESP_OK && !(e==ESP_ERR_NOT_FOUND && r.magic==SPI_MAGIC))return e;
    *extended=r.reserved==0xd1;*frame_valid=(r.flags&1)!=0;*latest_id=r.frame_id;return ESP_OK;
}
esp_err_t fpga_transport_live(fpga_live_status_t *s){
    if(CONFIG_STYLE_MOCK_SOURCE){portENTER_CRITICAL(&mock_lock);bool effect=esp_timer_get_time()<mock_effect_until;mock_live.effect_requested=mock_live.effect_actual=effect;mock_live.source_fps=CONFIG_STYLE_STREAM_FPS;*s=mock_live;portEXIT_CRITICAL(&mock_lock);return ESP_OK;}
    spi_header_t h={.opcode=7},r={0};esp_err_t e=command(&h,NULL,NULL,&r);if(e!=ESP_OK)return e;
    uint16_t v=(uint16_t)r.offset;
    if(r.reserved!=0xd1 || r.length || (r.offset>>16) || ((v>>5)&3)>2 || ((v>>7)&3)>2 || ((v>>14)&3)>2 ||
       (((v>>13)&1) && ((v&3)<1 || r.style!=(v&3))))return ESP_ERR_INVALID_RESPONSE;
    *s=(fpga_live_status_t){.extended=true,.frame_valid=(r.flags&1)!=0,.latest_frame_id=r.frame_id,.active_epoch=r.epoch,
      .active_style=v&3,.target_style=(v>>2)&3,.target_pending=(v>>4)&1,.requested_mode=(v>>5)&3,.actual_mode=(v>>7)&3,
      .fps_requested=(v>>9)&1,.fps_actual=(v>>10)&1,.effect_requested=(v>>11)&1,.effect_actual=(v>>12)&1,
      .model_valid=(v>>13)&1,.accepted_mode=(v>>14)&3,.source_fps=r.fps};return ESP_OK;
}
esp_err_t fpga_transport_remote(uint8_t action,uint8_t value,bool *uncertain){
    if(uncertain)*uncertain=false;
    if(action>4 || (action==1 && value>1) || (action==2 && value>2) || ((action==0 || action==3 || action==4) && value))return ESP_ERR_INVALID_ARG;
    if(CONFIG_STYLE_MOCK_SOURCE){portENTER_CRITICAL(&mock_lock);
      if(action==1)mock_live.fps_requested=mock_live.fps_actual=value;
      if(action==2 || action==3){uint8_t mode=action==2?value:(mock_live.requested_mode+1)%3;mock_live.requested_mode=mock_live.accepted_mode=mock_live.actual_mode=mode;}
      if(action==4){mock_effect_until=esp_timer_get_time()+2000000;mock_live.effect_requested=mock_live.effect_actual=true;}
      portEXIT_CRITICAL(&mock_lock);return ESP_OK;}
    spi_header_t h={.opcode=7,.flags=action,.style=value};return command_ex(&h,NULL,NULL,NULL,uncertain);
}
