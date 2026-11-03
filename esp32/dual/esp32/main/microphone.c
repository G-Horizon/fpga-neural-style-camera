#include "fpga_transport.h"
#include "sdkconfig.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "voice_local.h"
static portMUX_TYPE mic_mux=portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t lost_samples;
static bool IRAM_ATTR mic_overflow(i2s_chan_handle_t ch,i2s_event_data_t *event,void *ctx){
    (void)ch;(void)ctx;portENTER_CRITICAL_ISR(&mic_mux);lost_samples+=event->size/8;portEXIT_CRITICAL_ISR(&mic_mux);return false;
}
static void mic_task(void *arg) {
    (void)arg;
    i2s_chan_handle_t rx;
    i2s_chan_config_t cc=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,I2S_ROLE_MASTER);
    cc.dma_desc_num=8;cc.dma_frame_num=320;
    ESP_ERROR_CHECK(i2s_new_channel(&cc,NULL,&rx));
    i2s_std_config_t sc={.clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(16000),
      .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,I2S_SLOT_MODE_STEREO),
      .gpio_cfg={.mclk=I2S_GPIO_UNUSED,.bclk=18,.ws=16,.dout=I2S_GPIO_UNUSED,.din=17}};
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx,&sc));
    // SPH0645 tri-states the unselected slot. Use the S3's weak internal
    // pull-down as an initial no-extra-part idle level; qualify real PCM.
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_17,GPIO_PULLDOWN_ONLY));
    i2s_event_callbacks_t callbacks={.on_recv_q_ovf=mic_overflow};
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx,&callbacks,NULL));ESP_ERROR_CHECK(i2s_channel_enable(rx));
    int32_t stereo[640];int16_t mono[320];
#if CONFIG_VOICE_ON_S3
    voice_local_t recognizer;voice_local_init(&recognizer);
    ESP_LOGI("mic","SPH0645: 16kHz left slot, personal templates on ESP32-S3");
#else
    uint32_t seq=0,dropped=0;
    ESP_LOGI("mic","SPH0645: 16kHz left slot, personal templates on FPGA");
#endif
    for(;;){size_t got=0;esp_err_t e=i2s_channel_read(rx,stereo,sizeof(stereo),&got,1000);
      if(e!=ESP_OK || got==0){
#if CONFIG_VOICE_ON_S3
        voice_local_abort(&recognizer);
#else
        // Unknown-size read gaps still break the FPGA's continuity check.
        fpga_transport_audio_discontinuity(1);
#endif
        continue;
      }
      size_t n=got/(2*sizeof(int32_t));
      portENTER_CRITICAL(&mic_mux);uint32_t lost=lost_samples;lost_samples=0;portEXIT_CRITICAL(&mic_mux);
      if(lost){
#if CONFIG_VOICE_ON_S3
        voice_local_abort(&recognizer);
#else
        fpga_transport_audio_discontinuity(lost);
#endif
      }
      // SPH0645 SEL=GND selects left slot; its signed sample is MSB aligned.
      for(size_t i=0;i<n;i++)mono[i]=(int16_t)(stereo[2*i]>>16);
#if CONFIG_VOICE_ON_S3
      for(size_t i=0;i<n;i++){
        voice_local_event_t event;
        if(voice_local_feed(&recognizer,mono[i],&event) && event.accepted){
          e=fpga_transport_set_style(event.style_id,event.next);
          if(e!=ESP_OK)ESP_LOGW("mic","accepted command was not delivered: %s",esp_err_to_name(e));
          else ESP_LOGI("mic","personal command=%u score=%u margin=%u",event.command_class,event.score,event.second_score-event.score);
        }
      }
#else
      if(fpga_transport_audio(mono,n,seq++)!=ESP_OK && (++dropped%50)==1)ESP_LOGW("mic","FPGA audio not accepted; dropped blocks=%lu",(unsigned long)dropped);
#endif
    }
}
void microphone_start(void) {
    if(CONFIG_STYLE_MOCK_SOURCE)return;
    gpio_config_t sel={.pin_bit_mask=UINT64_C(1)<<15,.mode=GPIO_MODE_OUTPUT};ESP_ERROR_CHECK(gpio_config(&sel));ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_15,0));
    if(CONFIG_STYLE_MIC_ENABLE && xTaskCreate(mic_task,"microphone",6144,NULL,5,NULL)!=pdPASS)ESP_LOGE("mic","task allocation failed");
}
