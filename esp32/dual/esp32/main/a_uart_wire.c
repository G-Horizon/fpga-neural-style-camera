#include "a_uart_wire.h"
#include <string.h>
uint32_t d2_u32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
void d2_put32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(i*8));}
uint16_t d2_crc16(const uint8_t *p,size_t n){uint16_t c=0xffff;while(n--){c^=(uint16_t)*p++<<8;for(unsigned b=0;b<8;b++)c=(uint16_t)((c<<1)^((c&0x8000)?0x1021:0));}return c;}
size_t d2_uart_encode(uint8_t *out,size_t cap,const d2_uart_packet_t *p){
 if(!out||!p||p->length>D2_UART_MAX_PAYLOAD||cap<p->length+12u)return 0;
 out[0]=0xa5;out[1]=0x5a;out[2]=D2_UART_VERSION;out[3]=p->type;d2_put32(out+4,p->sequence);out[8]=(uint8_t)p->length;out[9]=(uint8_t)(p->length>>8);memcpy(out+10,p->payload,p->length);
 uint16_t c=d2_crc16(out+2,p->length+8);out[p->length+10]=(uint8_t)c;out[p->length+11]=(uint8_t)(c>>8);return p->length+12u;
}
static void drop(d2_uart_parser_t *s,size_t n){s->used-=n;memmove(s->data,s->data+n,s->used);s->discarded+=(uint32_t)n;}
bool d2_uart_feed(d2_uart_parser_t *s,uint8_t byte,uint32_t now,d2_uart_packet_t *out){
 if(s->used&&(uint32_t)(now-s->last_ms)>100){s->used=0;s->timeouts++;}s->last_ms=now;
 if(s->used==sizeof(s->data)){drop(s,1);}
 s->data[s->used++]=byte;
 for(;;){
  if(s->used<2)return false;
  if(s->data[0]!=0xa5||s->data[1]!=0x5a){drop(s,1);continue;}
  if(s->used<10)return false;
  uint16_t n=(uint16_t)(s->data[8]|(uint16_t)s->data[9]<<8);
  if(s->data[2]!=D2_UART_VERSION||n>D2_UART_MAX_PAYLOAD){s->length_errors++;drop(s,1);continue;}
  size_t total=n+12u;if(s->used<total)return false;
  uint16_t c=(uint16_t)(s->data[n+10]|(uint16_t)s->data[n+11]<<8);
  if(c!=d2_crc16(s->data+2,n+8)){s->crc_errors++;drop(s,1);continue;}
  out->type=s->data[3];out->sequence=d2_u32(s->data+4);out->length=n;memcpy(out->payload,s->data+10,n);drop(s,total);return true;
 }
}
static bool legal(uint8_t a,uint8_t v){return (a==1&&v<=2)||(a==2&&v==0)||(a==3&&v<=2)||(a==4&&v==0)||(a==5&&v<=1)||(a==6&&v==1);}
bool d2_control_begin(d2_control_host_t *s,uint8_t a,uint8_t v,uint32_t *id){
 if(!s->session||s->pending||!legal(a,v))return false;
 uint32_t next=s->last_id+1;if(!next)next=1;s->last_id=next;s->pending_id=next;s->action=a;s->value=v;s->pending=true;s->uncertain=false;*id=next;return true;
}
void d2_control_complete(d2_control_host_t *s,uint32_t session,uint32_t id){if(s->session==session&&s->pending&&s->pending_id==id){s->pending=false;s->uncertain=false;}}
void d2_control_timeout(d2_control_host_t *s){if(s->pending){s->pending=false;s->uncertain=true;}}
void d2_control_new_session(d2_control_host_t *s,uint32_t session){bool lost=s->uncertain||s->pending;memset(s,0,sizeof(*s));s->session=session;s->uncertain=lost;}
