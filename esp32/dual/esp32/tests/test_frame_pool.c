#include "frame_pool.h"
#include <assert.h>
#include <stdio.h>
int main(void){d2_frame_pool_t p={0};assert(!d2_pool_acquire(&p));assert(!d2_pool_release(&p));assert(d2_pool_begin(&p));assert(!d2_pool_begin(&p));assert(!d2_pool_acquire(&p));assert(d2_pool_publish(&p,true));for(int i=0;i<3;i++)assert(d2_pool_acquire(&p));assert(!d2_pool_acquire(&p));assert(!d2_pool_begin(&p));assert(!d2_pool_publish(&p,false));
 /* Two fast senders finish while one blocks: no producer may rewrite him. */
 assert(d2_pool_release(&p));assert(d2_pool_release(&p));for(int t=0;t<100;t++)assert(!d2_pool_begin(&p));assert(d2_pool_release(&p));
 /* Timeout has released the last ref. Waiting ACKs no longer hold the pool. */
 assert(d2_pool_begin(&p));assert(!d2_pool_acquire(&p));assert(d2_pool_publish(&p,false));assert(!d2_pool_acquire(&p));assert(d2_pool_begin(&p));assert(d2_pool_publish(&p,true));assert(d2_pool_acquire(&p));assert(d2_pool_release(&p));assert(!d2_pool_release(&p));
 d2_frame_credit_t c[3]={{.credit=true},{.credit=true},{.credit=true}};for(int i=0;i<3;i++){assert(d2_credit_begin(&c[i],1,0xabcdef01,10,100));assert(!d2_credit_begin(&c[i],2,0xabcdef01,11,200));}assert(!d2_credit_ack(&c[0],1,1,10));assert(!d2_credit_ack(&c[0],1,0xabcdef01,9));assert(d2_credit_ack(&c[0],1,0xabcdef01,10));assert(!d2_credit_ack(&c[0],1,0xabcdef01,10));assert(!d2_credit_eligible(&c[0],1,0xabcdef01,10));assert(d2_credit_begin(&c[0],4,0xabcdef01,13,400));assert(!d2_credit_expired(&c[2],14000100,14000000));assert(d2_credit_expired(&c[2],14000101,14000000));assert(d2_credit_ack(&c[1],1,0xabcdef01,10));assert(d2_credit_begin(&c[1],7,0xabcdef01,16,500));
 puts("PASS: actual production one-pool3reader/credit,slowreader nooverwrite,timeout14s/ACK32identity/staleACKreject/latestskip,ACKdoesnothold,failedcapture neverpublished");return 0;}
