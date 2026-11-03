#pragma once
#include "host_stubs.h"
typedef void *TaskHandle_t;
static inline int xTaskCreate(void(*fn)(void*),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *out){(void)fn;(void)name;(void)stack;(void)arg;(void)priority;if(out)*out=(void*)1;return pdPASS;}
static inline void xTaskNotifyGive(TaskHandle_t task){(void)task;}
static inline unsigned ulTaskNotifyTake(int clear,unsigned wait){(void)clear;(void)wait;return 0;}
