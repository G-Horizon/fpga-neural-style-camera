/* Integer personal-keyword fallback. No SDK calls, allocation, floating point,
 * external services or recognition enable here. Authoritative reference:
 * tools/voice_reference.py; training is the second round of the supplied audio.
 * The first round is a weak same-recording holdout, not generic voice validation.
 */
#include "voice_local.h"
#include <string.h>

static const int16_t coefficient_q14[8] = {32413,31786,30274,27246,20788,9512,-6393,-23170};
static const uint8_t training_lengths[4] = {35,44,41,40};
/* VOICE_LOCAL_GENERATED_TEMPLATES_BEGIN */
static const uint8_t templates[4][16][8] = {
    {
        {0,3,1,9,9,13,16,21},
        {0,0,1,5,6,13,16,21},
        {1,0,2,6,7,14,15,13},
        {0,3,6,5,10,12,13,11},
        {0,4,5,5,11,13,13,20},
        {0,4,4,7,9,11,14,19},
        {0,2,5,4,11,10,15,10},
        {0,2,2,3,12,11,18,18},
        {0,3,3,4,5,6,8,10},
        {4,0,2,9,7,13,22,22},
        {5,6,0,7,9,9,12,11},
        {4,4,0,5,6,8,8,10},
        {1,1,0,7,9,6,13,20},
        {3,0,2,5,5,9,9,10},
        {3,0,2,5,9,11,9,11},
        {0,0,0,7,8,1,7,15}
    },
    {
        {2,0,0,0,0,5,15,3},
        {0,4,3,5,4,7,16,4},
        {0,4,5,6,4,10,10,19},
        {0,6,8,15,8,16,15,23},
        {0,1,3,1,1,13,17,17},
        {3,3,3,2,0,4,7,5},
        {1,2,3,0,0,2,6,1},
        {0,2,5,6,7,11,11,11},
        {1,4,0,2,7,7,17,17},
        {1,0,2,3,1,2,16,16},
        {1,0,3,4,11,2,16,16},
        {0,0,0,2,3,5,7,8},
        {0,1,5,6,6,8,11,9},
        {0,6,8,12,8,15,14,22},
        {0,2,6,5,3,4,18,18},
        {0,1,2,6,0,7,14,8}
    },
    {
        {0,2,0,1,0,1,15,15},
        {0,2,6,0,0,2,2,15},
        {0,3,3,0,2,2,15,15},
        {2,5,2,0,3,4,5,16},
        {0,3,6,6,6,10,10,21},
        {0,3,7,7,9,18,15,16},
        {0,3,5,6,5,10,10,10},
        {0,4,4,5,4,9,13,19},
        {0,4,12,4,4,5,19,19},
        {0,3,9,10,11,6,20,20},
        {0,4,6,9,9,15,19,12},
        {2,0,5,10,10,9,20,20},
        {0,3,6,8,5,13,20,20},
        {1,0,10,13,16,14,15,22},
        {1,0,6,8,2,8,17,17},
        {1,0,0,2,6,5,6,14}
    },
    {
        {1,0,1,4,3,5,7,9},
        {0,0,2,1,4,6,4,16},
        {0,2,4,3,2,6,17,8},
        {0,2,2,6,6,7,8,8},
        {7,0,4,3,7,10,11,11},
        {4,8,0,1,7,11,15,22},
        {0,0,7,4,6,11,21,21},
        {0,7,7,8,13,14,17,23},
        {0,7,8,9,11,11,14,23},
        {0,4,7,7,9,10,13,12},
        {0,4,5,7,7,8,10,17},
        {0,3,5,4,4,9,19,19},
        {0,0,3,8,10,11,13,13},
        {0,3,4,6,8,11,14,12},
        {0,0,1,3,7,11,16,16},
        {3,0,1,1,8,13,11,16}
    }
};
/* VOICE_LOCAL_GENERATED_TEMPLATES_END */

/* Defined arithmetic floor, including negative products: do not rely on
 * implementation-defined right shifting of signed C integers. All products
 * fit int32 because recursive states saturate to +/-16383. */
static int32_t floor_shift(int32_t v, unsigned n) {
    if (v < 0) return -(((-v) + ((1L << n)-1)) >> n);
    return v >> n;
}
static uint8_t bit_length(uint32_t v) {
    uint8_t n=0;
    while(v){++n;v>>=1;}
    return n;
}
void voice_local_init(voice_local_t *ctx) { memset(ctx,0,sizeof(*ctx)); }
void voice_local_abort(voice_local_t *ctx) {
    memset(ctx->s1,0,sizeof(ctx->s1));memset(ctx->s2,0,sizeof(ctx->s2));
    ctx->amplitude_sum=0;ctx->sample_count=0;ctx->active=false;
    ctx->phrase_count=0;ctx->last_voiced_len=0;ctx->quiet=0;ctx->too_long=false;
}

static void match_phrase(voice_local_t *ctx, voice_local_event_t *event) {
    uint16_t scores[4],best=65535,second=65535;
    uint8_t best_class=0,n=ctx->last_voiced_len;
    memset(event,0,sizeof(*event));event->phrase_frames=n;
    event->score=event->second_score=65535;
    if(ctx->too_long||n<10||n>64){event->reject_reason=1;return;}
    for(unsigned c=0;c<4;c++) {
        if(n<training_lengths[c]/2||n>training_lengths[c]*2){scores[c]=65535;continue;}
        uint16_t score=0;
        for(unsigned t=0;t<16;t++) {
            unsigned f=((2*t+1)*n)>>5;
            for(unsigned b=0;b<8;b++) {
                int d=(int)ctx->phrase[f][b]-(int)templates[c][t][b];
                score+=(uint16_t)(d<0?-d:d);
            }
        }
        scores[c]=score;
    }
    for(unsigned c=0;c<4;c++) {
        if(scores[c]<best){second=best;best=scores[c];best_class=(uint8_t)c;}
        else if(scores[c]<second)second=scores[c];
    }
    event->command_class=best_class;event->score=best;event->second_score=second;
    if(best>650||(uint32_t)second<(uint32_t)best+40){event->reject_reason=2;return;}
    if(ctx->cooldown){event->reject_reason=3;return;}
    event->accepted=true;event->next=best_class==3;
    event->style_id=event->next?0:(uint8_t)(best_class+1);ctx->cooldown=31;
}

static bool process_frame(voice_local_t *ctx, voice_local_event_t *event) {
    const bool voiced=ctx->last_amplitude>80;
    if(ctx->cooldown)--ctx->cooldown;
    if(!ctx->active) {
        if(voiced) {
            ctx->active=true;ctx->too_long=false;ctx->quiet=0;
            ctx->phrase_count=ctx->last_voiced_len=1;
            memcpy(ctx->phrase[0],ctx->last_features,8);
        }
        return false;
    }
    const uint8_t before=ctx->phrase_count;
    if(before<64){memcpy(ctx->phrase[before],ctx->last_features,8);++ctx->phrase_count;}
    else if(voiced)ctx->too_long=true; /* VAD trailing silence uses no storage. */
    if(voiced){ctx->last_voiced_len=(uint8_t)(before+1);ctx->quiet=0;return false;}
    if(++ctx->quiet!=6)return false;
    ctx->active=false;ctx->quiet=0;match_phrase(ctx,event);return true;
}

bool voice_local_feed(voice_local_t *ctx, int16_t pcm, voice_local_event_t *event) {
    int32_t v=pcm;const int32_t scaled=floor_shift(v,6);
    ctx->amplitude_sum+=(uint32_t)(v<0?-v:v);
    for(unsigned b=0;b<8;b++) {
        int32_t s=scaled+floor_shift((int32_t)coefficient_q14[b]*ctx->s1[b],14)-ctx->s2[b];
        if(s>16383)s=16383;else if(s<-16383)s=-16383;
        ctx->s2[b]=ctx->s1[b];ctx->s1[b]=(int16_t)s;
    }
    if(++ctx->sample_count!=256)return false;
    uint8_t logs[8],maximum=0;
    for(unsigned b=0;b<8;b++) {
        const int32_t a=ctx->s1[b],z=ctx->s2[b];
        const int32_t cross=floor_shift(a*z,14);
        const int32_t power=a*a+z*z-(int32_t)coefficient_q14[b]*cross;
        logs[b]=bit_length(power>0?(uint32_t)power:0);
        if(logs[b]>maximum)maximum=logs[b];
        ctx->s1[b]=ctx->s2[b]=0;
    }
    for(unsigned b=0;b<8;b++)ctx->last_features[b]=(uint8_t)(maximum-logs[b]);
    ctx->last_amplitude=(uint16_t)(ctx->amplitude_sum>>8);++ctx->frames_seen;
    ctx->sample_count=0;ctx->amplitude_sum=0;
    return process_frame(ctx,event);
}
