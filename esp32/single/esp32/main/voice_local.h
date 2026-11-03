#ifndef VOICE_LOCAL_H
#define VOICE_LOCAL_H
#include <stdint.h>
#include <stdbool.h>

/* Personal template fallback, deliberately not enabled by this file.
 * Feed contiguous signed PCM16 at 16 kHz; input gain is calibrated externally.
 * A decision may be a rejection: only event.accepted is a style request.
 */
typedef struct {
    bool accepted;
    uint8_t command_class; /* 0 Vangogh, 1 Ukiyoe, 2 Inkwash, 3 next */
    uint8_t style_id;      /* physical IDs 1/2/3; ignored for next */
    bool next;
    uint8_t reject_reason; /* 0 accepted, 1 duration, 2 score/margin, 3 cooldown */
    uint16_t phrase_frames, score, second_score;
} voice_local_event_t;

typedef struct {
    int16_t s1[8], s2[8];
    uint32_t amplitude_sum, frames_seen;
    uint16_t sample_count, last_amplitude;
    uint8_t last_features[8], phrase[64][8];
    uint8_t phrase_count, last_voiced_len, quiet, cooldown;
    bool active, too_long;
} voice_local_t;

void voice_local_init(voice_local_t *ctx);
void voice_local_abort(voice_local_t *ctx);
bool voice_local_feed(voice_local_t *ctx, int16_t pcm, voice_local_event_t *event);
#endif
