#pragma once
#include "host_stubs.h"
typedef struct {unsigned capacity,count,head;void *items[2];} *QueueHandle_t;
static inline QueueHandle_t xQueueCreate(unsigned count,size_t bytes){assert(count==2&&bytes==sizeof(void*));QueueHandle_t q=calloc(1,sizeof(*q));q->capacity=count;return q;}
static inline int xQueueSend(QueueHandle_t q,const void *p,unsigned wait){(void)wait;if(q->count==q->capacity)return 0;q->items[(q->head+q->count)%q->capacity]=*(void *const*)p;q->count++;return pdTRUE;}
static inline int xQueueReceive(QueueHandle_t q,void *p,unsigned wait){(void)wait;if(!q->count)return 0;*(void**)p=q->items[q->head];q->head=(q->head+1)%q->capacity;q->count--;return pdTRUE;}
