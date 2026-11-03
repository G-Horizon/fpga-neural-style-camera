#pragma once
#include "esp_err.h"
#include "esp_http_server.h"
typedef unsigned (*remote_client_count_fn)(void);
// transport_ready describes driver initialization, not an FPGA connection.
esp_err_t remote_debug_start(httpd_handle_t server, bool transport_ready, remote_client_count_fn stream_clients);
