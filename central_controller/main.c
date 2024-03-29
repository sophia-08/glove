/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "app_error.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util.h"
#include "ble.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_nus_c.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_midi.h"
#include "app_usbd_string_desc.h"

#define APP_BLE_CONN_CFG_TAG 1  /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO 3 /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA 0 /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< BLE Nordic UART Service (NUS) client instances. */
NRF_BLE_GATT_DEF(m_gatt);                                         /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                  /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                         /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                  /**< BLE GATT Queue instance. */
    NRF_SDH_BLE_CENTRAL_LINK_COUNT,
    NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
    {
        .uuid = BLE_UUID_NUS_SERVICE,
        .type = NUS_SERVICE_UUID_TYPE};

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function to start scanning. */
static void scan_start(void) {
  ret_code_t ret;

  ret = nrf_ble_scan_start(&m_scan);
  APP_ERROR_CHECK(ret);

  ret = bsp_indication_set(BSP_INDICATE_SCANNING);
  APP_ERROR_CHECK(ret);
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const *p_scan_evt) {
  ret_code_t err_code;

  switch (p_scan_evt->scan_evt_id) {
  case NRF_BLE_SCAN_EVT_CONNECTING_ERROR: {
    err_code = p_scan_evt->params.connecting_err.err_code;
    APP_ERROR_CHECK(err_code);
  } break;

  case NRF_BLE_SCAN_EVT_CONNECTED: {
    ble_gap_evt_connected_t const *p_connected =
        p_scan_evt->params.connected.p_connected;

    NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
        p_connected->peer_addr.addr[0],
        p_connected->peer_addr.addr[1],
        p_connected->peer_addr.addr[2],
        p_connected->peer_addr.addr[3],
        p_connected->peer_addr.addr[4],
        p_connected->peer_addr.addr[5]);
  } break;

  case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
    NRF_LOG_INFO("Scan timed out.");
    scan_start();
  } break;

  default:
    break;
  }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void) {
  ret_code_t err_code;
  nrf_ble_scan_init_t init_scan;

  memset(&init_scan, 0, sizeof(init_scan));

  init_scan.connect_if_match = true;
  init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

  err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt) {
  ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */

void midi_send(uint8_t *msg, uint8_t len);

uint8_t sent_key_left;
uint8_t sent_key_right;
//uint8_t notes_righthand[5] = {60, 62, 64, 65, 67};
//uint8_t notes_lefthand[5] = {69, 71, 72, 74, 76};
typedef struct {
  int16_t h;
  int16_t r;
  int16_t p;
} imu_euler_t;

imu_euler_t right_euler;
imu_euler_t left_euler;

int8_t nature_pos[8][4] = {
{0x17,0x03,0xf3,0x05},  //piano
{0x3d,0xe4,0xd4,0x5d},  //guitar
{0x28,0xfd,0xe2,0x84},  //violin
{0x05,0x39,0xea,0x2a},  //trumpet
{0x03,0x2f,0xeb,0x2a},  //flute
{0x14,0x40,0xf8,0x37},  //homorica
{0x4b,0xaa,0xaa,0xaf},  //harp
{0x16,0xe1,0xcc,0x58},  //cello
};
#define ERROR_POS 20

uint8_t notes[7] = {60, 62, 64, 65, 67, 69, 71};
uint8_t notes_sent[7];

typedef enum {
  INSTRUMENT_PIANO,
  INSTRUMENT_GUITAR,
  INSTRUMENT_VIOLIN,
    INSTRUMENT_TRUMPET,
  INSTRUMENT_FLUTE,
  INSTRUMENT_HOMORICA,
  INSTRUMENT_HARP,
  INSTRUMENT_CELLO
} instrument_t;

instrument_t instrument;
int scale=0;



// entry point of receive remote data.
static void ble_nus_chars_received_uart_print(uint8_t *p_data, uint16_t data_len) {
  ret_code_t ret_val;

  //  NRF_LOG_DEBUG("Receiving data.");
//  NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
  uint8_t deviceId = p_data[0] & 0x0f;
  uint8_t keys = p_data[1];
//  NRF_LOG_DEBUG("%d, %x, %x", deviceId, keys, sent_key_right, sent_key_left);

  if (deviceId == 0) {
    // right hand
    uint8_t i;

    right_euler.h = (int8_t)p_data[2];
    right_euler.r = (int8_t)p_data[3];
    right_euler.p = (int8_t)p_data[4];

    if (p_data[0] & 0x80) {
      // select key pressed
        int8_t i=0;
        instrument = 0;
        for (i=0; i<8; i++) {
          if ( (abs(right_euler.r - nature_pos[i][0]) < ERROR_POS) &&
                (abs(right_euler.p - nature_pos[i][1]) < ERROR_POS) &&
                (abs(left_euler.r - nature_pos[i][2]) < ERROR_POS) &&
                (abs(left_euler.p - nature_pos[i][3]) < ERROR_POS) )
                instrument = i;
        }

//      if ((right_euler.r < 15 && right_euler.r > -15) && (right_euler.p < 15 && right_euler.p > -15)) {
//        instrument = INSTRUMENT_PIANO;
//      } else if ((right_euler.r < 0x55 && right_euler.r > 0x25) && (right_euler.p < 26 && right_euler.p > -26)) {
//        instrument = INSTRUMENT_GUITAR;
//      } else if ((right_euler.r < 15 && right_euler.r > -15) && (right_euler.p < 0X50 && right_euler.p > 0X30)) {
//        instrument = INSTRUMENT_TRUMPET;
//      } else if ((right_euler.r < 0X38 && right_euler.r > 0X16) && (right_euler.p < 40 && right_euler.p > 0)) {
//        instrument = INSTRUMENT_VIOLIN;
//      } else if ((right_euler.r < 0X20 && right_euler.r > 0) && (right_euler.p < -70 && right_euler.p > -100)) {
//        instrument = INSTRUMENT_FLUTE;
//      } else {
//        instrument = INSTRUMENT_PIANO;
//      }
      NRF_LOG_DEBUG("Select %d, right hand euler: %x, %x, %x %x", instrument, (uint8_t)right_euler.r, (uint8_t)right_euler.p,(uint8_t)left_euler.r, (uint8_t)left_euler.p);
//NRF_LOG_DEBUG("Select %d", instrument);
    }

    if (keys == sent_key_right) {
      return;
    }
    for (i = 0; i < 5; i++) {  // thumb, pointer, middle, ring, pinkie
      if ((keys & (1 << i)) != (sent_key_right & (1 << i))) {
        if (keys & (1 << i)) {
          //down
          uint8_t message[3] = {0x90 | instrument, notes[i]+scale, 50};
          notes_sent[i] = notes[i]+scale;
          //  byte 1: bit0-3 channel id
          //  byte 2: note id
          //  byte 3: velocity
          midi_send(message, sizeof(message));
          NRF_LOG_DEBUG("press %d", notes[i]+scale);
        } else {
          // release
          uint8_t message[3] = {0x80 | instrument, notes_sent[i], 50};
          midi_send(message, sizeof(message));
          NRF_LOG_DEBUG("relese %d", notes_sent[i]);
        }
      }
    }
    sent_key_right = keys;
  } else {
    //left hand

    left_euler.h = (int8_t)p_data[2];
    left_euler.r = (int8_t)p_data[3];
    left_euler.p = (int8_t)p_data[4];

    uint8_t i;
    if (keys & (1 << 4)) { //pinkie finger down
      if (keys & (1 << 2)) { // both pinkie and middle down, scale -12
        scale = -12; //
      } else {
        scale = -1; //Only pinkie down, scale - 1
      }
    }else if (keys & (1 << 2)) { //middle finger down
      if (keys & (1 << 3)) { // both middle and ring down, scale +12
        scale = 12; //
      } else {
        scale = 1; //Only middle down, scale + 1
      }
    } else {
      scale =0;
    }
//    NRF_LOG_DEBUG("octane %d", scale);

    if (keys == sent_key_left) {
      return;
    }
    for (i = 0; i < 2; i++) {
      if ((keys & (1 << i)) != (sent_key_left & (1 << i))) {
        if (keys & (1 << i)) {
          //down
          uint8_t message[3] = {0x90 | instrument, notes[i+5]+scale, 50};
          notes_sent[i+5] = notes[i+5]+scale;
          midi_send(message, sizeof(message));
          NRF_LOG_DEBUG("press %d", notes[i+5]+scale);
        } else {
          // release
          uint8_t message[3] = {0x80 | instrument, notes_sent[i+5], 50};
          midi_send(message, sizeof(message));
          NRF_LOG_DEBUG("relese %d", notes_sent[i+5]);
        }
      }
    }
    sent_key_left = keys;
  }

  return;

  for (uint32_t i = 0; i < data_len; i++) {
    do {
      ret_val = app_uart_put(p_data[i]);
      if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY)) {
        NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
        APP_ERROR_CHECK(ret_val);
      }
    } while (ret_val == NRF_ERROR_BUSY);
  }
  if (p_data[data_len - 1] == '\r') {
    while (app_uart_put('\n') == NRF_ERROR_BUSY)
      ;
  }
  if (ECHOBACK_BLE_UART_DATA) {
    for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++) {
      // Send data back to the peripheral.
      do {
        ret_val = ble_nus_c_string_send(&m_ble_nus_c[c], p_data, data_len);
        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) && (ret_val != NRF_ERROR_INVALID_STATE)) {
          NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
          APP_ERROR_CHECK(ret_val);
        }
      } while (ret_val == NRF_ERROR_BUSY);
    }
  }
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t *p_event) {
  static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
  static uint16_t index = 0;
  uint32_t ret_val;

  switch (p_event->evt_type) {
  /**@snippet [Handling data from UART] */
  case APP_UART_DATA_READY:
    UNUSED_VARIABLE(app_uart_get(&data_array[index]));
    index++;

    if ((data_array[index - 1] == '\n') ||
        (data_array[index - 1] == '\r') ||
        (index >= (m_ble_nus_max_data_len))) {
      NRF_LOG_DEBUG("Ready to send data over BLE NUS");
      NRF_LOG_HEXDUMP_DEBUG(data_array, index);

      for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++) {
        do {
          ret_val = ble_nus_c_string_send(&m_ble_nus_c[c], data_array, index);
          if ((ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES)) {
            APP_ERROR_CHECK(ret_val);
          }
        } while (ret_val == NRF_ERROR_RESOURCES);
      }
      index = 0;
    }
    break;

  /**@snippet [Handling data from UART] */
  case APP_UART_COMMUNICATION_ERROR:
    NRF_LOG_ERROR("Communication error occurred while handling UART.");
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;

  case APP_UART_FIFO_ERROR:
    NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t *p_ble_nus_c, ble_nus_c_evt_t const *p_ble_nus_evt) {
  ret_code_t err_code;

  switch (p_ble_nus_evt->evt_type) {
  case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
    NRF_LOG_INFO("Discovery complete.");
    err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
    APP_ERROR_CHECK(err_code);

    err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Connected to device with Nordic UART Service.");
    break;

  case BLE_NUS_C_EVT_NUS_TX_EVT: //receive data from remote
    ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
    break;

  case BLE_NUS_C_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected.");
    scan_start();
    break;
  }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event) {
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  switch (event) {
  case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;
  }

  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code;
  ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_ble_evt->evt.gap_evt.conn_handle], p_ble_evt->evt.gap_evt.conn_handle, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);

    // start discovery of services. The NUS Client waits for a discovery result
    err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
    APP_ERROR_CHECK(err_code);

    if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
      // Resume scanning.
      scan_start();
    }
    break;

  case BLE_GAP_EVT_DISCONNECTED:

    NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
        p_gap_evt->conn_handle,
        p_gap_evt->params.disconnected.reason);
    break;

  case BLE_GAP_EVT_TIMEOUT:
    if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
      NRF_LOG_INFO("Connection Request timed out.");
    }
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    // Pairing not supported.
    err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    // Accepting parameters requested by peer.
    err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
        &p_gap_evt->params.conn_param_update_request.conn_params);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    NRF_LOG_INFO("ATT MTU exchange completed.");

    m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void) {
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event) {
  ret_code_t err_code;

  switch (event) {
  case BSP_EVENT_SLEEP:
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    break;

  case BSP_EVENT_DISCONNECT:
    for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++) {
      err_code = sd_ble_gap_disconnect(m_ble_nus_c[c].conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;

  default:
    break;
  }
}

/**@brief Function for initializing the UART. */
static void uart_init(void) {
  ret_code_t err_code;

  app_uart_comm_params_t const comm_params =
      {
          .rx_pin_no = RX_PIN_NUMBER,
          .tx_pin_no = TX_PIN_NUMBER,
          .rts_pin_no = RTS_PIN_NUMBER,
          .cts_pin_no = CTS_PIN_NUMBER,
          .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
          .use_parity = false,
          .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200};

  APP_UART_FIFO_INIT(&comm_params,
      UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      uart_event_handle,
      APP_IRQ_PRIORITY_LOWEST,
      err_code);

  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void) {
  ret_code_t err_code;
  ble_nus_c_init_t init;

  init.evt_handler = ble_nus_c_evt_handler;
  init.error_handler = nus_error_handler;
  init.p_gatt_queue = &m_ble_gatt_queue;
  for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++) {
    err_code = ble_nus_c_init(&m_ble_nus_c[c], &init);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void) {
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the timer. */
static void timer_init(void) {
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void) {
  ble_db_discovery_init_t db_init;

  memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

  db_init.evt_handler = db_disc_handler;
  db_init.p_gatt_queue = &m_ble_gatt_queue;

  ret_code_t err_code = ble_db_discovery_init(&db_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

int main(void) {

  ret_code_t ret;

  // Initialize.
  log_init();
  ret = nrf_drv_clock_init();
  APP_ERROR_CHECK(ret);
  nrf_drv_clock_lfclk_request(NULL);
  while (!nrf_drv_clock_lfclk_is_running()) {
    /* Just waiting */
  }

  timer_init();
  uart_init();
  //  buttons_leds_init();

  usb_midi_init_dev();
  db_discovery_init();
  power_management_init();
  ble_stack_init();
  gatt_init();
  nus_c_init();
  ble_conn_state_init();
  scan_init();
  // Start execution.
  printf("Glove central controller started.\r\n");
  NRF_LOG_INFO("Glove central controller started.");
  scan_start();
  ret = app_usbd_power_events_enable();
  APP_ERROR_CHECK(ret);

  // Enter main loop.

  for (;;) {

    while (app_usbd_event_queue_process()) {
      /* Nothing to do */
    }
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    //        idle_state_handle();
    //        __WFE();
  }
}