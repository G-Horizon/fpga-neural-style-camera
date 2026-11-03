#include "remote_control.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

typedef struct { const char *p, *end; } parser_t;
static void ws(parser_t *p) { while(p->p<p->end && (*p->p==' ' || *p->p=='\t' || *p->p=='\r' || *p->p=='\n')) p->p++; }
static bool take(parser_t *p, char c) { ws(p); if(p->p==p->end || *p->p!=c)return false; p->p++; return true; }
static int hex(char c) { return c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:c>='A'&&c<='F'?c-'A'+10:-1; }
// IDs are printable ASCII. Escaped printable ASCII is decoded before deduplication.
static bool string(parser_t *p, char *out, size_t cap) {
    if(!take(p,'"'))return false;
    size_t n=0;
    while(p->p<p->end) { unsigned char c=(unsigned char)*p->p++;
        if(c=='"') { out[n]=0; return true; }
        if(c=='\\') { if(p->p==p->end)return false; c=(unsigned char)*p->p++;
            if(c=='u') { unsigned v=0; for(int i=0;i<4;i++) { if(p->p==p->end)return false; int h=hex(*p->p++); if(h<0)return false; v=v*16+(unsigned)h; } if(v>126)return false; c=(unsigned char)v; }
            else if(c!='"' && c!='\\' && c!='/')return false;
        }
        if(c<32 || c>126 || n+1>=cap)return false;
        out[n++]=(char)c;
    } return false;
}
static bool literal(parser_t *p, const char *s) { size_t n=strlen(s); if((size_t)(p->end-p->p)<n || memcmp(p->p,s,n))return false; p->p+=n; return true; }
bool remote_has_value(remote_command_t c) { return c==REMOTE_STYLE || c==REMOTE_MODE || c==REMOTE_FPS; }
const char *remote_command_name(remote_command_t c) { static const char *names[]={"style","next","mode","mode_next","fps","effect"}; return c<=REMOTE_EFFECT?names[c]:"invalid"; }
const char *remote_state_name(remote_state_t s) { static const char *names[]={"queued","sent","failed","uncertain"}; return s<=REMOTE_UNCERTAIN?names[s]:"invalid"; }
bool remote_parse(const char *body, size_t bytes, remote_request_t *out, const char **error) {
    *error="invalid_json";
    if(!body || !out || bytes==0 || bytes>REMOTE_BODY_MAX)return false;
    parser_t p={body,body+bytes}; remote_request_t r={0}; char command[16]={0}; unsigned seen=0; int type=0;
    if(!take(&p,'{'))return false;
    for(;;) { char key[24]; if(!string(&p,key,sizeof(key)) || !take(&p,':'))return false;
        unsigned bit=!strcmp(key,"request_id")?1:!strcmp(key,"command")?2:!strcmp(key,"value")?4:0;
        if(!bit || (seen&bit)) { *error="unknown_or_duplicate_field"; return false; } seen|=bit;
        if(bit==1) { if(!string(&p,r.request_id,sizeof(r.request_id)) || !r.request_id[0]) { *error="invalid_request_id"; return false; } }
        else if(bit==2) { if(!string(&p,command,sizeof(command)))return false; }
        else { ws(&p);
            if(literal(&p,"true")) { r.value=1; type=2; }
            else if(literal(&p,"false")) { r.value=0; type=2; }
            else if(literal(&p,"null")) { type=0; }
            else { type=1; bool neg=false; if(p.p<p.end && *p.p=='-') { neg=true; p.p++; }
                if(p.p==p.end || *p.p<'0' || *p.p>'9')return false;
                if(*p.p=='0' && p.p+1<p.end && p.p[1]>='0' && p.p[1]<='9')return false;
                unsigned v=0; while(p.p<p.end && *p.p>='0' && *p.p<='9') { v=v*10+(unsigned)(*p.p++-'0'); if(v>100)return false; } r.value=neg?-(int)v:(int)v;
            }
        }
        if(take(&p,'}'))break;
        if(!take(&p,','))return false;
    }
    ws(&p); if(p.p!=p.end || (seen&3)!=3)return false;
    bool found=false; for(int i=REMOTE_STYLE;i<=REMOTE_EFFECT;i++)if(!strcmp(command,remote_command_name((remote_command_t)i))) { r.command=(remote_command_t)i; found=true; break; }
    if(!found) { *error="unknown_command"; return false; }
    if((r.command==REMOTE_STYLE && (type!=1 || r.value<1 || r.value>3)) ||
       (r.command==REMOTE_MODE && (type!=1 || r.value<0 || r.value>2)) ||
       (r.command==REMOTE_FPS && type!=2) || (!remote_has_value(r.command) && type!=0)) { *error="invalid_value"; return false; }
    if(!remote_has_value(r.command))r.value=0;
    *out=r; *error=NULL; return true;
}
void remote_control_init(remote_control_t *c) { memset(c,0,sizeof(*c)); }
int remote_submit(remote_control_t *c, const remote_request_t *r, bool connected, bool extended, int64_t now, int *index, const char **error) {
    *index=-1; *error=NULL;
    for(int i=0;i<REMOTE_RECORDS;i++)if(c->records[i].used && !strcmp(c->records[i].request.request_id,r->request_id)) {
        remote_record_t *v=&c->records[i]; *index=i;
        if(v->request.command==r->command && v->request.value==r->value)return 200;
        *error="request_id_conflict"; return 409;
    }
    if(!connected) { *error="fpga_offline"; return 503; }
    if(!extended && r->command!=REMOTE_STYLE && r->command!=REMOTE_NEXT) { *error="extended_controls_unavailable"; return 422; }
    int slot=-1; uint32_t oldest=UINT32_MAX;
    for(int i=0;i<REMOTE_RECORDS;i++) { remote_record_t *v=&c->records[i]; if(!v->used) { slot=i; break; } if(v->state!=REMOTE_QUEUED && v->order<oldest) { oldest=v->order; slot=i; } }
    if(slot<0 || c->count==REMOTE_RECORDS) { *error="command_queue_full"; return 503; }
    remote_record_t *v=&c->records[slot]; memset(v,0,sizeof(*v)); v->used=true; v->request=*r; v->state=REMOTE_QUEUED; v->order=++c->order; v->created_at_ms=now;
    snprintf(v->message,sizeof(v->message),"Queued; FPGA has not acknowledged this command");
    c->queue[(c->head+c->count)%REMOTE_RECORDS]=(uint8_t)slot; c->count++; *index=slot; return 202;
}
int remote_dequeue(remote_control_t *c) { if(!c->count)return -1; int i=c->queue[c->head]; c->head=(c->head+1)%REMOTE_RECORDS; c->count--; c->records[i].dispatching=true; return i; }
void remote_complete(remote_control_t *c,int i,remote_state_t s,int64_t now,const char *error,const char *message) {
    if(i<0 || i>=REMOTE_RECORDS || !c->records[i].used)return;
    remote_record_t *v=&c->records[i]; v->state=s; v->dispatching=false; v->completed_at_ms=now;
    snprintf(v->error,sizeof(v->error),"%s",error?error:""); snprintf(v->message,sizeof(v->message),"%s",message?message:"");
}
