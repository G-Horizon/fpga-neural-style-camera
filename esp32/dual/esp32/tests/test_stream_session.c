#include "frame_pool.h"
#include <assert.h>
#include <stdio.h>
typedef struct{uint32_t id,epoch;bool renew;} example_t;
int main(int argc,char**argv){
 /* No transport error between these complete snapshots, including a fast
  * board reboot, either-coordinate regression, ambiguity and legal wrap. */
 const example_t v[]={{500,100,false},{501,100,false},{1,0,true},{2,0,false},{3,1,false},{2,2,true},{3,1,true},{0x80000003,1,true},{0xfffffffe,0xfffffffe,true},{0xffffffff,0xffffffff,false},{0,0,false},{1,0,false},{1,0,false}};
 uint32_t session=0x12340001,previous_id=0,previous_epoch=0,sequence=0xfffffff8;FILE*out=NULL;char path[2048];
 if(argc>1){snprintf(path,sizeof(path),"%s/session_cases.json",argv[1]);out=fopen(path,"wb");assert(out);fputs("{\"scope\":\"NO_SPI_ERROR_COMPLETE_CRC_RELEASE_SNAPSHOTS\",\"frames\":[",out);}
 for(unsigned i=0;i<sizeof(v)/sizeof(v[0]);i++){
  bool renew=i&&d2_stream_identity_regressed(previous_id,previous_epoch,v[i].id,v[i].epoch);assert(renew==v[i].renew);
  if(renew)session++;sequence++;
  if(out)fprintf(out,"%s{\"id\":%u,\"epoch\":%u,\"sequence\":%u,\"session\":%u,\"renewed\":%s}",i?",":"",v[i].id,v[i].epoch,sequence,session,renew?"true":"false");
  previous_id=v[i].id;previous_epoch=v[i].epoch;
 }
 if(out){fputs("]}\n",out);fclose(out);}
 assert(!d2_stream_identity_regressed(0xffffffff,0xffffffff,0,0));
 assert(!d2_stream_identity_regressed(2,8,2,8));
 assert(d2_stream_identity_regressed(2,8,1,9));
 assert(d2_stream_identity_regressed(2,8,3,7));
 /* Old sender reference must retire before a reset capture can touch data. */
 d2_frame_pool_t p={0};d2_frame_credit_t c={.credit=true};assert(d2_pool_begin(&p));assert(d2_pool_publish(&p,true));assert(d2_pool_acquire(&p));assert(d2_credit_begin(&c,500,100,10,0));assert(!d2_pool_begin(&p));assert(d2_pool_release(&p));
 /* A failed partial snapshot is not published and cannot be sent. The caller
  * retains its previous identity/session, matching capture's e==OK guard. */
 assert(d2_pool_begin(&p));assert(d2_pool_publish(&p,false));assert(!p.published&&!d2_pool_acquire(&p));
 /* Valid reset snapshot can publish with no old image references, while the
  * old ACK remains pending. Do not force credit or overlap complete frames. */
 assert(d2_pool_begin(&p));assert(d2_pool_publish(&p,true));assert(!d2_credit_eligible(&c,1,0,11));assert(!d2_credit_ack(&c,1,0,11));assert(d2_credit_ack(&c,500,100,10));assert(d2_credit_begin(&c,1,0,11,100));assert(!d2_credit_ack(&c,500,100,10));assert(d2_credit_ack(&c,1,0,11));
 /* ACK2 has no session field: preserve the global S3 publication sequence.
  * Even repeated FID/epoch cannot make an old ACK release a newer frame. */
 assert(d2_credit_begin(&c,1,0,12,200));assert(!d2_credit_ack(&c,1,0,11));assert(!d2_credit_eligible(&c,2,0,13));assert(d2_credit_ack(&c,1,0,12));assert(d2_credit_begin(&c,2,0,13,300));
 puts("PASS: complete-snapshot fast-reset FID/epoch regression renews session without SPIerror;equal/forward/legal32wrap stable;oldsender reference+ACK retired before newflow;oldACK cannot release newsequence;13Cpublicationcases");return 0;
}
