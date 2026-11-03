#pragma once
#include "host_stubs.h"
typedef struct {unsigned num;} wifi_sta_list_t;
static inline esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t *list){list->num=4;return ESP_OK;}
