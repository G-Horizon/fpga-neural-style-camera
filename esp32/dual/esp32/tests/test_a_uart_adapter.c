#include "a_uart.h"
#include "a_uart_wire.h"
#include "driver/uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t reply_bytes[512];static size_t ready,pos;static uint32_t sensor_session,next_audio,control_id,epoch=0xfedcba98,random_seed=20;
static unsigned controls,pcm_packets,pcm_samples,hellos,aborts;static uint32_t last_abort;static bool drop,corrupt,bad_layout,reject_pcm,continuous;static uint8_t requested=2;static int16_t samples_seen[1024];
uint32_t esp_random(void){return ++random_seed;}
esp_err_t uart_param_config(int p,const uart_config_t*c){assert(p==1&&c->baud_rate==1000000&&c->data_bits==8&&c->stop_bits==1);return ESP_OK;}
esp_err_t uart_set_pin(int p,int tx,int rx,int rts,int cts){assert(p==1&&tx==4&&rx==5&&rts==-1&&cts==-1);return ESP_OK;}
esp_err_t uart_driver_install(int p,int rx,int tx,int depth,void*q,int flags){assert(p==1&&rx==4096&&tx==0&&depth==0&&!q&&!flags);return ESP_OK;}
int uart_write_bytes(int port,const void *data,size_t size){
 assert(port==1);d2_uart_parser_t parser={0};d2_uart_packet_t p,r={.length=40};bool valid=false;for(size_t i=0;i<size;i++)valid|=d2_uart_feed(&parser,((const uint8_t*)data)[i],(uint32_t)i,&p);assert(valid);r.type=p.type|0x80;r.sequence=p.sequence;uint8_t status=0;
 if(p.type==D2_HELLO){assert(p.length==4&&d2_u32(p.payload));sensor_session=d2_u32(p.payload);next_audio=0;control_id=0;continuous=false;hellos++;}
 else if(d2_u32(p.payload)!=sensor_session||!sensor_session)status=2;
 else if(p.type==D2_CONTROL){assert(p.length==6&&p.sequence&&(!control_id||(int32_t)(p.sequence-control_id)>0));control_id=p.sequence;controls++;if(p.payload[4]==1)requested=p.payload[5];if(p.payload[4]==2)requested=(requested+1)%3;epoch++;}
 else if(p.type==D2_PCM_ABORT){assert(p.length==8);last_abort=d2_u32(p.payload+4);next_audio+=last_abort;continuous=false;aborts++;}
 else if(p.type==D2_PCM){assert(p.length>=10&&p.length<=128&&!(p.length&1));unsigned n=(p.length-8)/2;uint32_t ix=d2_u32(p.payload+4);if(reject_pcm){reject_pcm=false;status=5;}else{assert(!continuous||ix==next_audio);continuous=true;for(unsigned i=0;i<n;i++)samples_seen[pcm_samples++]=(int16_t)(p.payload[8+2*i]|p.payload[9+2*i]<<8);pcm_packets++;next_audio=ix+n;}}
 else assert(p.type==D2_STATUS&&p.length==4);
 d2_put32(r.payload,sensor_session);r.payload[4]=status;r.payload[5]=r.payload[6]=requested;r.payload[7]=2;r.payload[8]=1;r.payload[9]=0;r.payload[10]=1;r.payload[11]=0;d2_put32(r.payload+12,epoch);d2_put32(r.payload+16,0xa1234567);d2_put32(r.payload+20,0xa1234566);d2_put32(r.payload+24,0xa1234565);d2_put32(r.payload+28,next_audio);d2_put32(r.payload+32,0);r.payload[36]=0;r.payload[37]=4;r.payload[38]=7;
 if(bad_layout){r.payload[7]=3;bad_layout=false;}assert(pos==ready);pos=ready=0;if(!drop){ready=d2_uart_encode(reply_bytes,sizeof(reply_bytes),&r);if(corrupt){reply_bytes[ready-1]^=0x10;corrupt=false;}}return(int)size;
}
int uart_read_bytes(int p,void*out,uint32_t n,unsigned ticks){assert(p==1);size_t take=ready-pos;if(take>n)take=n;if(take){memcpy(out,reply_bytes+pos,take);pos+=take;return(int)take;}if(ticks)Sleep(1);return 0;}
esp_err_t uart_get_buffered_data_len(int p,size_t*n){assert(p==1);*n=ready-pos;return ESP_OK;}
int main(void){assert(a_uart_init()==ESP_OK&&hellos==1);a_uart_status_t s;assert(a_uart_status(&s)==ESP_OK);assert(s.session==sensor_session&&s.active_style==2&&s.requested_epoch==0xfedcba98&&s.capture_fid==0xa1234567&&s.pcm_free==1024&&s.capabilities==7);
 bool uncertain;assert(a_uart_control(2,0,&uncertain)==ESP_OK&&!uncertain&&controls==1&&requested==0);assert(a_uart_control(5,2,&uncertain)==ESP_ERR_INVALID_ARG&&controls==1);
 drop=true;assert(a_uart_control(2,0,&uncertain)==ESP_ERR_TIMEOUT&&uncertain&&controls==2);drop=false;assert(a_uart_status(&s)==ESP_OK&&controls==2);assert(a_uart_control(1,2,&uncertain)==ESP_OK&&!uncertain&&controls==3);
 corrupt=true;assert(a_uart_control(2,0,&uncertain)==ESP_ERR_TIMEOUT&&uncertain&&controls==4);assert(a_uart_status(&s)==ESP_OK&&controls==4);bad_layout=true;assert(a_uart_control(2,0,&uncertain)==ESP_ERR_INVALID_RESPONSE&&uncertain&&controls==5);assert(a_uart_status(&s)==ESP_OK&&controls==5);sensor_session=0;assert(a_uart_status(&s)==ESP_OK&&hellos==2&&controls==5);
 int16_t input[320];for(unsigned i=0;i<320;i++)input[i]=(int16_t)(i*213-32000);assert(a_uart_audio(input,320)==ESP_OK&&pcm_packets==6&&pcm_samples==320&&next_audio==320&&aborts==1);assert(!memcmp(input,samples_seen,sizeof(input)));
 reject_pcm=true;assert(a_uart_audio(input,120)==ESP_ERR_NO_MEM&&pcm_samples==320);a_uart_discontinuity(7);assert(a_uart_audio(input,120)==ESP_OK&&last_abort==127&&aborts==2&&pcm_samples==440&&next_audio==567);assert(!memcmp(input,samples_seen+320,240));
 puts("PASS: production UART adapter1Mbps GPIO4/5,ACK40/FID32/session,CRC/lostACK uncertain no replay,newHELLO,PCM6pack exactS16,no duplicate,space rejection/127sample abort/rejoin");return 0;}
