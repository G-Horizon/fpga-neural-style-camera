#pragma once
#include "host_stubs.h"
typedef void *httpd_handle_t;
typedef struct {size_t content_len;const char *content_type,*body;size_t consumed;char response[16384],status[64];bool close,no_store;int recv_calls,recv_fail;bool asynchronous;} httpd_req_t;
typedef struct {const char *uri;int method;esp_err_t (*handler)(httpd_req_t*);} httpd_uri_t;
#define HTTP_GET 0
#define HTTP_POST 1
#define HTTPD_SOCK_ERR_TIMEOUT -3
#define HTTPD_RESP_USE_STRLEN -1
static inline void httpd_resp_set_status(httpd_req_t *r,const char *s){snprintf(r->status,sizeof(r->status),"%s",s);}
static inline void httpd_resp_set_type(httpd_req_t *r,const char *s){(void)r;(void)s;}
static inline void httpd_resp_set_hdr(httpd_req_t *r,const char *key,const char *v){if(!strcmp(key,"Connection"))r->close=!strcmp(v,"close");if(!strcmp(key,"Cache-Control"))r->no_store=!strcmp(v,"no-store");}
static inline esp_err_t httpd_resp_send(httpd_req_t *r,const char *s,int n){(void)n;snprintf(r->response,sizeof(r->response),"%s",s);return ESP_OK;}
static inline size_t httpd_req_get_hdr_value_len(httpd_req_t *r,const char *key){(void)key;return r->content_type?strlen(r->content_type):0;}
static inline esp_err_t httpd_req_get_hdr_value_str(httpd_req_t *r,const char *key,char *out,size_t cap){(void)key;snprintf(out,cap,"%s",r->content_type);return ESP_OK;}
static inline int httpd_req_recv(httpd_req_t *r,char *out,size_t bytes){r->recv_calls++;if(r->recv_fail)return r->recv_fail;size_t remaining=r->content_len-r->consumed;if(bytes>remaining)bytes=remaining;if(bytes>7)bytes=7;memcpy(out,r->body+r->consumed,bytes);r->consumed+=bytes;return (int)bytes;}
extern int host_async_begun,host_async_completed,host_registered;
static inline esp_err_t httpd_req_async_handler_begin(httpd_req_t *r,httpd_req_t **copy){r->asynchronous=true;*copy=r;host_async_begun++;return ESP_OK;}
static inline esp_err_t httpd_req_async_handler_complete(httpd_req_t *r){assert(r->asynchronous);r->asynchronous=false;host_async_completed++;return ESP_OK;}
static inline esp_err_t httpd_register_uri_handler(httpd_handle_t s,const httpd_uri_t *u){(void)s;(void)u;host_registered++;return ESP_OK;}
