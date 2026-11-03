#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Caller holds its own mutex. No allocation; one shared image, at most3senders. */
typedef struct {unsigned refs;bool published,capturing;} d2_frame_pool_t;
bool d2_pool_begin(d2_frame_pool_t *p);
bool d2_pool_publish(d2_frame_pool_t *p,bool complete);
bool d2_pool_acquire(d2_frame_pool_t *p);
bool d2_pool_release(d2_frame_pool_t *p);
typedef struct {bool credit;uint32_t id,epoch,sequence;int64_t sent_us;}d2_frame_credit_t;
bool d2_credit_eligible(const d2_frame_credit_t *c,uint32_t id,uint32_t epoch,uint32_t sequence);
bool d2_credit_begin(d2_frame_credit_t *c,uint32_t id,uint32_t epoch,uint32_t sequence,int64_t now);
bool d2_credit_ack(d2_frame_credit_t *c,uint32_t id,uint32_t epoch,uint32_t sequence);
bool d2_credit_expired(const d2_frame_credit_t *c,int64_t now,int64_t timeout);
/* Compare only complete, CRC-checked and RELEASE-successful snapshots.
 * Equal identities may be a repeated snapshot/quality change; legal mod32
 * forward wraps keep the stream session. Half-ring ambiguity renews it. */
bool d2_stream_identity_regressed(uint32_t previous_id,uint32_t previous_epoch,uint32_t id,uint32_t epoch);
