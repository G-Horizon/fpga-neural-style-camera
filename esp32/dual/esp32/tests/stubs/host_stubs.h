#pragma once
#define WIN32_LEAN_AND_MEAN
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#include <windows.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_TIMEOUT 1
#define ESP_ERR_NOT_FOUND 2
#define ESP_ERR_INVALID_RESPONSE 3
#define ESP_ERR_INVALID_CRC 4
#define ESP_ERR_INVALID_STATE 5
#define ESP_ERR_INVALID_SIZE 6
#define ESP_ERR_INVALID_ARG 7
#define ESP_ERR_NOT_SUPPORTED 8
#define ESP_ERR_NO_MEM 9
#define ESP_ERROR_CHECK(x) assert((x)==ESP_OK)
static inline void host_log(const char *tag,const char *format,...){(void)tag;(void)format;}
#define ESP_LOGW(...) host_log(__VA_ARGS__)
#define ESP_LOGI(...) host_log(__VA_ARGS__)
static inline const char *esp_err_to_name(esp_err_t e){return e==ESP_OK?"ESP_OK":e==ESP_ERR_TIMEOUT?"ESP_ERR_TIMEOUT":e==ESP_ERR_NOT_SUPPORTED?"ESP_ERR_NOT_SUPPORTED":"ESP_FAIL";}
typedef SRWLOCK portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED SRWLOCK_INIT
#define portENTER_CRITICAL(p) AcquireSRWLockExclusive(p)
#define portEXIT_CRITICAL(p) ReleaseSRWLockExclusive(p)
typedef HANDLE SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void){return CreateMutex(NULL,FALSE,NULL);}
static inline int xSemaphoreTake(HANDLE h,unsigned ms){return WaitForSingleObject(h,ms)==WAIT_OBJECT_0;}
static inline void xSemaphoreGive(HANDLE h){assert(ReleaseMutex(h));}
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY INFINITE
#define pdMS_TO_TICKS(x) (x)
static inline void vTaskDelay(unsigned ticks){Sleep(ticks);}
static inline void esp_rom_delay_us(unsigned us){(void)us;Sleep(0);}
static inline int64_t esp_timer_get_time(void){LARGE_INTEGER counter,freq;QueryPerformanceCounter(&counter);QueryPerformanceFrequency(&freq);return counter.QuadPart*1000000/freq.QuadPart;}
#define MALLOC_CAP_DMA 1
#define MALLOC_CAP_INTERNAL 2
static inline void *heap_caps_calloc(size_t n,size_t s,int caps){(void)caps;return calloc(n,s);}
typedef int spi_device_handle_t;
typedef struct {size_t length;const void *tx_buffer;void *rx_buffer;} spi_transaction_t;
typedef struct {int mosi_io_num,miso_io_num,sclk_io_num,quadwp_io_num,quadhd_io_num,max_transfer_sz;} spi_bus_config_t;
typedef struct {int clock_speed_hz,mode,spics_io_num,queue_size,cs_ena_pretrans,cs_ena_posttrans;} spi_device_interface_config_t;
#define SPI2_HOST 2
#define SPI_DMA_CH_AUTO 1
esp_err_t spi_device_polling_transmit(spi_device_handle_t device,spi_transaction_t *transaction);
static inline esp_err_t spi_bus_initialize(int host,const spi_bus_config_t *b,int dma){(void)host;(void)b;(void)dma;return ESP_OK;}
static inline esp_err_t spi_bus_add_device(int host,const spi_device_interface_config_t *d,spi_device_handle_t *out){(void)host;(void)d;*out=1;return ESP_OK;}
typedef struct {uint64_t pin_bit_mask;int mode;} gpio_config_t;
#define GPIO_MODE_INPUT 1
static inline esp_err_t gpio_config(const gpio_config_t *g){(void)g;return ESP_OK;}
int gpio_get_level(int pin);
