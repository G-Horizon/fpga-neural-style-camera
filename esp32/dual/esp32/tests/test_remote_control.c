#include "remote_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void){static remote_control_t c;remote_request_t r;const char *e;int i;remote_control_init(&c);
 const char *good="{\"request_id\":\"same\",\"command\":\"next\"}";assert(remote_parse(good,strlen(good),&r,&e));assert(remote_submit(&c,&r,true,true,1,&i,&e)==202&&c.count==1);int original=i;assert(remote_submit(&c,&r,true,true,2,&i,&e)==200&&i==original&&c.count==1);r.command=REMOTE_EFFECT;assert(remote_submit(&c,&r,true,true,3,&i,&e)==409);r.command=REMOTE_NEXT;i=remote_dequeue(&c);remote_complete(&c,i,REMOTE_UNCERTAIN,4,"lostACK","Do not retry");assert(remote_submit(&c,&r,true,true,5,&i,&e)==200&&c.records[i].state==REMOTE_UNCERTAIN&&c.count==0);
 for(unsigned n=0;n<40;n++){snprintf(r.request_id,sizeof(r.request_id),"id-%u",n);assert(remote_submit(&c,&r,true,true,n+10,&i,&e)==202);int q=remote_dequeue(&c);assert(q==i);remote_complete(&c,i,REMOTE_SENT,n+11,NULL,"A accepted");}
 strcpy(r.request_id,"same");assert(remote_submit(&c,&r,true,true,100,&i,&e)==409&&!strcmp(e,"request_id_retired")&&c.count==0);
 while(c.accepted_count<REMOTE_RETIRED_IDS){snprintf(r.request_id,sizeof(r.request_id),"fill-%u",c.accepted_count);assert(remote_submit(&c,&r,true,true,0,&i,&e)==202);i=remote_dequeue(&c);remote_complete(&c,i,REMOTE_SENT,0,NULL,NULL);}
 strcpy(r.request_id,"beyond");assert(remote_submit(&c,&r,true,true,0,&i,&e)==503&&!strcmp(e,"request_history_full"));
 const char *bad[]={"{\"request_id\":\"a\",\"command\":\"style\",\"value\":0}","{\"request_id\":\"a\",\"command\":\"fps\",\"value\":1}","{\"request_id\":\"a\",\"command\":\"effect\",\"value\":1}","{\"request_id\":\"a\",\"request_id\":\"b\",\"command\":\"next\"}"};for(unsigned n=0;n<sizeof(bad)/sizeof(bad[0]);n++)assert(!remote_parse(bad[n],strlen(bad[n]),&r,&e));
 puts("PASS: productionHTTP strictcommands/idempotency/conflict/uncertain/cacheevictiontombstone/1024boundedhistoryfailclosed/noautoresend");return 0;}
