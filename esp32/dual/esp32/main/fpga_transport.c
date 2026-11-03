#include "fpga_transport.h"
#include "a_uart_wire.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
static spi_device_handle_t device;static SemaphoreHandle_t bus_lock;static uint8_t *tx,*rx;static uint32_t sequence,mock_fid,mock_token;static fpga_live_status_t mock_live={.extended=true,.model_valid=true,.active_style=1,.target_style=1,.fps_actual=true,.source_fps=60};
static esp_err_t transfer(void){spi_transaction_t t={.length=4096,.tx_buffer=tx,.rx_buffer=rx};return spi_device_polling_transmit(device,&t);}
static esp_err_t command_until(d2_spi_header_t *h,const uint8_t *payload,uint8_t *out,d2_spi_header_t *reply,int64_t deadline){
 if(xSemaphoreTake(bus_lock,pdMS_TO_TICKS(150))!=pdTRUE)return ESP_ERR_TIMEOUT;
 if(deadline&&esp_timer_get_time()>=deadline){xSemaphoreGive(bus_lock);return ESP_ERR_TIMEOUT;}
 h->magic=D2_SPI_MAGIC;h->version=D2_SPI_VERSION;h->sequence=++sequence;h->payload_crc32=payload?d2_crc32(payload,h->length):0;h->header_crc32=d2_crc32((const uint8_t*)h,44);
 memset(tx,0,D2_SPI_PACKET);memcpy(tx,h,sizeof(*h));if(payload)memcpy(tx+48,payload,h->length);
 esp_err_t e=transfer();int64_t end=esp_timer_get_time()+100000;if(deadline&&deadline<end)end=deadline;
 if(e==ESP_OK){esp_rom_delay_us(5);while(!gpio_get_level(14)){if(esp_timer_get_time()>end){e=ESP_ERR_TIMEOUT;break;}esp_rom_delay_us(2);}}
 if(e==ESP_OK){d2_spi_header_t nop={.magic=D2_SPI_MAGIC,.version=D2_SPI_VERSION};nop.header_crc32=d2_crc32((const uint8_t*)&nop,44);memset(tx,0,D2_SPI_PACKET);memcpy(tx,&nop,48);e=transfer();}
 if(e==ESP_OK){d2_spi_header_t r;memcpy(&r,rx,48);
  if(r.magic!=D2_SPI_MAGIC||r.version!=D2_SPI_VERSION||r.opcode!=h->opcode||r.sequence!=h->sequence||r.length>D2_SPI_DATA||r.reserved||r.header_crc32!=d2_crc32(rx,44))e=ESP_ERR_INVALID_RESPONSE;
  else if(r.payload_crc32!=d2_crc32(rx+48,r.length))e=ESP_ERR_INVALID_CRC;
  else{if(reply)*reply=r;if(r.status)e=r.status==1?ESP_ERR_NOT_FOUND:r.status==2?ESP_ERR_INVALID_STATE:ESP_FAIL;else if(out)memcpy(out,rx+48,r.length);}
 }
 xSemaphoreGive(bus_lock);return e;
}
static esp_err_t command(d2_spi_header_t *h,const uint8_t *payload,uint8_t *out,d2_spi_header_t *reply){return command_until(h,payload,out,reply,0);}
static esp_err_t caps(d2_spi_header_t *r){uint8_t p[D2_SPI_DATA];d2_spi_header_t h={.opcode=D2_SPI_CAPS};esp_err_t e=command(&h,NULL,p,r);if(e!=ESP_OK)return e;
 if(r->length!=16||d2_u32(p)!=0x1f||d2_u32(p+4)!=5000||d2_u32(p+12)||r->flags&~0x1f||r->style>2||r->effect>1||r->layout>2)return ESP_ERR_INVALID_RESPONSE;
 return ESP_OK;}
esp_err_t fpga_transport_init(void){
 ESP_ERROR_CHECK(a_uart_init());if(CONFIG_STYLE_MOCK_SOURCE){ESP_LOGW("D2SP","SIMULATION SOURCE; nophysicalSPI/UART");return ESP_OK;}
 bus_lock=xSemaphoreCreateMutex();if(!bus_lock)return ESP_ERR_NO_MEM;tx=heap_caps_calloc(1,512,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);rx=heap_caps_calloc(1,512,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);if(!tx||!rx)return ESP_ERR_NO_MEM;
 gpio_config_t g={.pin_bit_mask=UINT64_C(1)<<14,.mode=GPIO_MODE_INPUT};ESP_ERROR_CHECK(gpio_config(&g));
 spi_bus_config_t b={.mosi_io_num=11,.miso_io_num=13,.sclk_io_num=12,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=512};ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&b,SPI_DMA_CH_AUTO));
 spi_device_interface_config_t d={.clock_speed_hz=CONFIG_STYLE_SPI_CLOCK_MHZ*1000000,.mode=0,.spics_io_num=10,.queue_size=1,.cs_ena_pretrans=1,.cs_ena_posttrans=1};return spi_bus_add_device(SPI2_HOST,&d,&device);
}
esp_err_t fpga_transport_query(fpga_frame_t *f){
 if(CONFIG_STYLE_MOCK_SOURCE){*f=(fpga_frame_t){.frame_id=mock_fid,.style_epoch=mock_live.active_epoch,.style_id=mock_live.active_style-1,.effect=mock_live.effect_actual,.source_fps_x100=6000};return ESP_OK;}
 d2_spi_header_t r;esp_err_t e=caps(&r);if(e!=ESP_OK)return e;if(!(r.flags&1))return ESP_ERR_NOT_FOUND;
 if(r.style>2||r.effect>1||r.layout>2)return ESP_ERR_INVALID_RESPONSE;
 *f=(fpga_frame_t){.frame_id=r.frame_id,.style_epoch=r.epoch,.style_id=r.style,.effect=r.effect,.source_fps_x100=r.fps*100};return ESP_OK;
}
esp_err_t fpga_transport_acquire(fpga_frame_t *f,bool hd){
 if(CONFIG_STYLE_MOCK_SOURCE){a_uart_status_t a;if(a_uart_status(&a)==ESP_OK){mock_live.active_style=a.active_style+1;mock_live.active_epoch=a.requested_epoch;mock_live.effect_actual=a.effect;}*f=(fpga_frame_t){.frame_id=++mock_fid,.style_epoch=mock_live.active_epoch,.style_id=mock_live.active_style-1,.effect=mock_live.effect_actual,.source_fps_x100=6000,.token=++mock_token,.hd=hd};return ESP_OK;}
 uint8_t payload[D2_SPI_DATA];d2_spi_header_t h={.opcode=D2_SPI_ACQUIRE,.flags=hd?1:0},r;esp_err_t e=command(&h,NULL,payload,&r);if(e!=ESP_OK)return e;
 fpga_frame_t lease={.frame_id=r.frame_id,.style_epoch=r.epoch,.style_id=r.style,.effect=r.effect,.source_fps_x100=r.fps*100,.token=r.lease,.hd=hd};
 d2_spi_meta_t m;memcpy(&m,payload,sizeof(m));
 if(r.length!=32||!r.lease||!(r.flags&1)||r.style>2||r.effect>1||m.style_bytes!=STREAM_PAYLOAD_BYTES||m.style_width!=256||m.style_height!=144||m.reserved||m.format_flags!=(hd?0x103u:1u)||m.guide_bytes!=(hd?GUIDE_PAYLOAD_BYTES:0u)||m.guide_width!=(hd?1920:0)||m.guide_height!=(hd?1080:0)||(!hd&&m.guide_crc32)||((r.flags&2)!=0)!=hd){if(r.lease)fpga_transport_release(&lease);return ESP_ERR_INVALID_RESPONSE;}
 lease.style_crc32=m.style_crc32;lease.guide_crc32=m.guide_crc32;lease.deadline_us=esp_timer_get_time()+4500000;*f=lease;return ESP_OK;
}
esp_err_t fpga_transport_read_chunk(const fpga_frame_t *f,uint8_t plane,size_t offset,uint8_t *out,size_t bytes){
 size_t total=plane==0?STREAM_PAYLOAD_BYTES:plane==1&&f->hd?GUIDE_PAYLOAD_BYTES:0;if(!total||offset>total||bytes>total-offset)return ESP_ERR_INVALID_SIZE;
 if(CONFIG_STYLE_MOCK_SOURCE){for(size_t i=0;i<bytes;i++){size_t q=offset+i;if(plane==1)out[i]=(uint8_t)((q%1920)/8+(q/1920)/4+f->frame_id);else out[i]=(uint8_t)(q%3==0?(q/3)%256:q%3==1?(q/768)*255/143:f->style_id*80+f->frame_id*3);}return ESP_OK;}
 while(bytes){uint16_t n=bytes>D2_SPI_DATA?D2_SPI_DATA:(uint16_t)bytes;uint8_t p[D2_SPI_DATA];d2_spi_header_t h={.opcode=D2_SPI_READ,.lease=f->token,.frame_id=f->frame_id,.epoch=f->style_epoch,.offset=(uint32_t)offset,.length=n,.plane=plane},r;esp_err_t e=command_until(&h,NULL,p,&r,f->deadline_us);if(e!=ESP_OK)return e;
  if(r.lease!=f->token||r.frame_id!=f->frame_id||r.epoch!=f->style_epoch||r.offset!=offset||r.length!=n||r.plane!=plane||r.style!=f->style_id||r.effect!=f->effect||r.plane_bytes!=total){return ESP_ERR_INVALID_RESPONSE;}
  memcpy(out,p,n);out+=n;offset+=n;bytes-=n;
 }return ESP_OK;
}
esp_err_t fpga_transport_release(const fpga_frame_t *f){if(CONFIG_STYLE_MOCK_SOURCE)return ESP_OK;d2_spi_header_t h={.opcode=D2_SPI_RELEASE,.lease=f->token,.frame_id=f->frame_id,.epoch=f->style_epoch},r;esp_err_t e=command(&h,NULL,NULL,&r);if(e==ESP_OK&&(r.lease!=f->token||r.frame_id!=f->frame_id||r.epoch!=f->style_epoch||r.length))return ESP_ERR_INVALID_RESPONSE;return e;}
esp_err_t fpga_transport_probe(bool *extended,bool *valid,uint32_t *fid){fpga_frame_t f;esp_err_t e=fpga_transport_query(&f);*extended=e==ESP_OK||e==ESP_ERR_NOT_FOUND;*valid=e==ESP_OK;*fid=e==ESP_OK?f.frame_id:0;return e==ESP_ERR_NOT_FOUND?ESP_OK:e;}
esp_err_t fpga_transport_live(fpga_live_status_t *s){
 a_uart_status_t a={0};esp_err_t ae=a_uart_status(&a);if(CONFIG_STYLE_MOCK_SOURCE){*s=mock_live;s->a_state=a;s->frame_valid=mock_fid!=0;s->latest_frame_id=mock_fid;return ESP_OK;}
 d2_spi_header_t r;esp_err_t e=caps(&r);if(e!=ESP_OK)return e;
 if(r.style>2||r.effect>1||r.layout>2)return ESP_ERR_INVALID_RESPONSE;
 *s=(fpga_live_status_t){.extended=true,.frame_valid=(r.flags&1)!=0,.model_valid=(r.flags&4)!=0,.latest_frame_id=r.frame_id,.active_epoch=r.epoch,.active_style=r.style+1,.target_style=ae==ESP_OK?a.requested_style+1:0,.target_pending=ae==ESP_OK&&(a.requested_epoch!=r.epoch||a.requested_style!=r.style),.requested_mode=ae==ESP_OK?a.layout:r.layout,.accepted_mode=ae==ESP_OK?a.layout:r.layout,.actual_mode=r.layout,.fps_requested=ae==ESP_OK?a.fps!=0:(r.flags&8)!=0,.fps_actual=(r.flags&8)!=0,.effect_requested=ae==ESP_OK?a.effect!=0:r.effect!=0,.effect_actual=r.effect!=0,.source_fps=r.fps,.a_state=a};return ESP_OK;
}
esp_err_t fpga_transport_remote_style(uint8_t style,bool next,bool *uncertain){if(!next&&(style<1||style>3))return ESP_ERR_INVALID_ARG;return a_uart_control(next?2:1,next?0:style-1,uncertain);}
esp_err_t fpga_transport_set_style(uint8_t style,bool next){return fpga_transport_remote_style(style,next,NULL);}
esp_err_t fpga_transport_remote(uint8_t action,uint8_t value,bool *uncertain){if(action>4)return ESP_ERR_INVALID_ARG;if(action==0){a_uart_status_t s;return a_uart_status(&s);}return a_uart_control(action==1?5:action==2?3:action==3?4:6,action==4?1:value,uncertain);}
esp_err_t fpga_transport_audio(const int16_t *samples,size_t n,uint32_t sequence){(void)sequence;return a_uart_audio(samples,n);}
void fpga_transport_audio_discontinuity(uint32_t n){a_uart_discontinuity(n);}
