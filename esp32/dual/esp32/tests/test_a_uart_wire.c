#include "a_uart_wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static bool feed(d2_uart_parser_t *s,const uint8_t *p,size_t n,uint32_t *t,d2_uart_packet_t *r){bool hit=false;for(size_t i=0;i<n;i++){bool v=d2_uart_feed(s,p[i],(*t)++,r);assert(!hit||!v);hit|=v;}return hit;}
int main(void){
 assert(d2_crc16((const uint8_t*)"123456789",9)==0x29b1);
 d2_uart_packet_t p={.type=D2_PCM,.sequence=0x89abcdef,.length=128},r;for(size_t i=0;i<128;i++)p.payload[i]=(uint8_t)(i*47);p.payload[7]=0xa5;p.payload[8]=0x5a;
 uint8_t b[140],bad[140];size_t n=d2_uart_encode(b,sizeof(b),&p);assert(n==140);assert(b[4]==0xef&&b[7]==0x89&&b[8]==128);assert(d2_uart_encode(b,139,&p)==0);
 d2_uart_parser_t s={0};uint32_t t=0;assert(feed(&s,b,n,&t,&r));assert(r.type==p.type&&r.sequence==p.sequence&&r.length==p.length&&!memcmp(p.payload,r.payload,128));
 for(unsigned cut=0;cut<n;cut++){memset(&s,0,sizeof(s));for(unsigned i=0;i<cut;i++)assert(!d2_uart_feed(&s,b[i],i,&r));t=1000;assert(feed(&s,b,n,&t,&r));assert(!cut||s.timeouts==1);}
 memset(&s,0,sizeof(s));t=0;memcpy(bad,b,n);bad[72]^=0x80;assert(!feed(&s,bad,n,&t,&r));assert(feed(&s,b,n,&t,&r));assert(s.crc_errors==1);
 memset(&s,0,sizeof(s));t=0;memcpy(bad,b,n);bad[8]=129;assert(!feed(&s,bad,n,&t,&r));assert(feed(&s,b,n,&t,&r));assert(s.length_errors>=1);
 memset(&s,0,sizeof(s));t=0;uint8_t noise[]={0,0xa5,0xa5,0x5a,9,0,0,0,0,0,0xff,0xff};assert(!feed(&s,noise,sizeof(noise),&t,&r));assert(feed(&s,b,n,&t,&r));
 memset(&s,0,sizeof(s));t=0xffffff80;assert(feed(&s,b,n,&t,&r));assert(feed(&s,b,n,&t,&r));assert(s.timeouts==0);
 d2_control_host_t h={0};uint32_t id;assert(!d2_control_begin(&h,2,0,&id));d2_control_new_session(&h,5);assert(d2_control_begin(&h,2,0,&id)&&id==1);assert(!d2_control_begin(&h,6,1,&id));d2_control_complete(&h,6,1);assert(h.pending);d2_control_complete(&h,5,2);assert(h.pending);d2_control_complete(&h,5,1);assert(!h.pending&&!h.uncertain);
 assert(d2_control_begin(&h,6,1,&id)&&id==2);d2_control_timeout(&h);assert(!h.pending&&h.uncertain&&h.last_id==2);d2_control_new_session(&h,6);assert(h.uncertain&&h.last_id==0&&!h.pending);assert(d2_control_begin(&h,1,2,&id)&&id==1);assert(!d2_control_begin(&h,5,2,&id));d2_control_complete(&h,6,1);h.last_id=0xffffffff;assert(d2_control_begin(&h,3,2,&id)&&id==1);
 puts("PASS: CCITT-knownvector,128payload/LE,140 partial-timeout boundaries,resync/CRC/length/noise/wrap,singlepending/session/uncertain/nonreplay/controlid");return 0;
}
