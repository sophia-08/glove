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

#define TWI_INSTANCE_ID 1

struct bno055_t bno055;
s32 comres = BNO055_ERROR;
/*************read raw Euler data************/
/* variable used to read the euler h data */
s16 euler_data_h = BNO055_INIT_VALUE;

/* variable used to read the euler r data */
s16 euler_data_r = BNO055_INIT_VALUE;

/* variable used to read the euler p data */
s16 euler_data_p = BNO055_INIT_VALUE;

/* structure used to read the euler hrp data */
struct bno055_euler_t euler_hrp;

/* Number of possible TWI addresses. */
// #define TWI_ADDRESSES      127
/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

__STATIC_INLINE void data_handler(uint8_t temp) {
  NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  switch (p_event->type) {
  case NRF_DRV_TWI_EVT_DONE:
    if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {
      //                data_handler(m_sample);
    }
    m_xfer_done = true;
    break;
  default:
    break;
  }
}

/**
 * @brief TWI initialization.
 */
#ifdef BOARD_PCA10059
#define ARDUINO_SCL_PIN 45
#define ARDUINO_SDA_PIN 47
#endif
void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = ARDUINO_SCL_PIN,
      .sda = ARDUINO_SDA_PIN,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
  //  nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));

  nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(dev_addr, &reg_addr, 1, reg_data, cnt);
  uint32_t flags = 0;
  m_xfer_done = false;
  ret_code_t ret = nrf_drv_twi_xfer(&m_twi, &xfer, flags);
  while (m_xfer_done == false)
    ;
  //nrf_delay_ms(200);
}

/*  \Brief: The API is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *  will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
  u8 data[32]; //For demo, assumed cnt is less than 32
  data[0] = reg_addr;
  m_xfer_done = false;
  memcpy(&data[1], reg_data, cnt);
  nrf_drv_twi_tx(&m_twi, dev_addr, data, cnt + 1, false);
  while (m_xfer_done == false)
    ;
  //nrf_delay_ms(200);
}

void BNO055_msleep(u32 cnt) {
  nrf_delay_ms(cnt);
}

s16 BNO055_readHeading() {
  bno055_read_euler_h(&euler_data_h);
  return euler_data_h;
}

s8 BNO055_readCalStatus() {
  s8 data;
  bno055_get_calib_stat(&data);
  return data;
}

void init_imu() {
  bno055.dev_addr = 0x28;
  bno055.bus_read = BNO055_I2C_bus_read;
  bno055.bus_write = BNO055_I2C_bus_write;
  bno055.delay_msec = BNO055_msleep;
  //
  comres = bno055_init(&bno055);

  /* set the power mode as NORMAL*/
  comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}