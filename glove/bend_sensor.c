#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_saadc.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include <stdint.h>
#include <string.h>
#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif

#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bno055.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"

extern uint8_t key[10];
#define THRESHOLD 150

static nrf_saadc_value_t adc_buf[2];


#define SAMPLES_IN_BUFFER 1
volatile uint8_t state = 1;

static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter;

static uint8_t m_adc_channel_enabled;
static nrf_saadc_channel_config_t channel_config[4];
//static nrf_saadc_channel_config_t channel_1_config;
//static nrf_saadc_channel_config_t channel_2_config;

void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
  if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
    static ret_code_t err_code;

    /*
    1. read the output
    2. uninit current channel
    3. init next channel. 0->1->2->3->0
    4. covert
    */

    key[m_adc_channel_enabled] = p_event->data.done.p_buffer[0] > THRESHOLD ? 0 : 1;

    err_code = nrf_drv_saadc_channel_uninit(m_adc_channel_enabled);
    APP_ERROR_CHECK(err_code);
    m_adc_channel_enabled++;
    if (m_adc_channel_enabled == 4) {
      m_adc_channel_enabled = 0;
    }
    err_code = nrf_drv_saadc_channel_init(m_adc_channel_enabled, &channel_config[m_adc_channel_enabled]);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

  }
}

void saadc_init(void) {
  static ret_code_t err_code;

  //set configuration for saadc channels
  uint8_t i;
  for (i = 0; i < 4; i++) {

    channel_config[i].resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config[i].resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_config[i].gain = NRF_SAADC_GAIN1_6;
    channel_config[i].reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_config[i].acq_time = NRF_SAADC_ACQTIME_3US;
    channel_config[i].mode = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_config[i].pin_n = NRF_SAADC_INPUT_DISABLED;
  }

  channel_config[0].pin_p = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);
  channel_config[1].pin_p = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN5);
  channel_config[2].pin_p = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN2);
  channel_config[3].pin_p = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0);

  err_code = nrf_drv_saadc_init(NULL, saadc_callback);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_channel_init(0, &channel_config[0]);
  APP_ERROR_CHECK(err_code);
  m_adc_channel_enabled = 0;

  //Configure TIMER mode and sampling rate of 200kHz
  NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos);
  NRF_SAADC->SAMPLERATE |= (80 << SAADC_SAMPLERATE_CC_Pos);

  // at Oversample rate 128, final NRF_DRV_SAADC_EVT_DONE rate is 200k/256 = 1.56k
  NRF_SAADC->OVERSAMPLE = NRF_SAADC_OVERSAMPLE_128X;

  err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

  //Start sampling. The internal SAADC timer will then trigger sampling until STOP task is called.
  nrf_drv_saadc_sample();
}