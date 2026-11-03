#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { char *buffer;size_t capacity,length;bool failed,first[8];unsigned depth; } remote_json_t;
void remote_json_init(remote_json_t *json,char *buffer,size_t capacity);
void remote_json_begin(remote_json_t *json,const char *key,char type);
void remote_json_end(remote_json_t *json,char type);
void remote_json_string(remote_json_t *json,const char *key,const char *value);
void remote_json_number(remote_json_t *json,const char *key,int64_t value);
void remote_json_bool(remote_json_t *json,const char *key,bool value);
void remote_json_null(remote_json_t *json,const char *key);
