#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REMOTE_RECORDS 16
#define REMOTE_BODY_MAX 192
#define REMOTE_ID_MAX 40
#define REMOTE_RETIRED_IDS 1024
typedef enum { REMOTE_STYLE, REMOTE_NEXT, REMOTE_MODE, REMOTE_MODE_NEXT, REMOTE_FPS, REMOTE_EFFECT } remote_command_t;
typedef enum { REMOTE_QUEUED, REMOTE_SENT, REMOTE_FAILED, REMOTE_UNCERTAIN } remote_state_t;
typedef struct { char request_id[REMOTE_ID_MAX+1]; remote_command_t command; int value; } remote_request_t;
typedef struct {
    remote_request_t request;
    remote_state_t state;
    bool used, dispatching;
    uint32_t order;
    int64_t created_at_ms, completed_at_ms;
    char error[48], message[128];
} remote_record_t;
typedef struct {
    remote_record_t records[REMOTE_RECORDS];
    uint8_t queue[REMOTE_RECORDS], head, count;
    uint32_t order;
    uint64_t accepted_ids[REMOTE_RETIRED_IDS];
    uint16_t accepted_count;
} remote_control_t;
// Pure C core. The caller owns synchronization; no allocation or I/O occurs here.
bool remote_parse(const char *body, size_t bytes, remote_request_t *out, const char **error);
void remote_control_init(remote_control_t *control);
// Returns HTTP status. Duplicates return 200 and the original record.
int remote_submit(remote_control_t *control, const remote_request_t *request,
                  bool connected, bool extended, int64_t now_ms, int *record, const char **error);
int remote_dequeue(remote_control_t *control);
void remote_complete(remote_control_t *control, int index, remote_state_t state,
                     int64_t now_ms, const char *error, const char *message);
const char *remote_command_name(remote_command_t command);
const char *remote_state_name(remote_state_t state);
bool remote_has_value(remote_command_t command);
