/**
 * Copyright (c) 2013 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_app_ant_hrs_main main.c
 * @{
 * @ingroup ble_sdk_app_ant_hrs
 * @brief HRM sample application using both BLE and ANT.
 *
 * The application uses the BLE Heart Rate Service (and also the Device Information
 * services), and the ANT HRM RX profile.
 *
 * It will open a receive channel which will connect to an ANT HRM TX profile device when the
 * application starts. The received data will be propagated to a BLE central through the
 * BLE Heart Rate Service.
 *
 * The ANT HRM TX profile device simulator SDK application
 * (Board\pca10003\ant\ant_hrm\hrm_tx_buttons) can be used as a peer ANT device. By changing
 * ANT_HRMRX_NETWORK_KEY to the ANT+ Network Key, the application will instead be able to connect to
 * an ANT heart rate belt.
 *
 * @note The ANT+ Network Key is available for ANT+ Adopters. Please refer to
 *       http://thisisant.com to become an ANT+ Adopter and access the key.
 *
 * @note This application is based on the BLE Heart Rate Service Sample Application
 *       (Board\nrf6310\ble\ble_app_hrs). Please refer to this application for additional
 *       documentation.
 */

#include <atomic>
#include <stdint.h>
#include <string.h>
#include <cmath>
#include <algorithm>

#include "nrf.h"
#include "nrf_gpio.h"

#include <nrf_dfu_ble_svci_bond_sharing.h>
#include <nrf_svci_async_function.h>
#include <nrf_svci_async_handler.h>


#include "nordic_common.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_conn_state.h"
#include "ble_conn_params.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include <ble_dfu.h>
#include "bsp.h"
#include "fds.h"
#include "led_softblink.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sensorsim.h"
#include "nrf_ble_gatt.h"
#include "nrfx_ppi.h"
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"
#include "nrf_sdh.h"
#include <nrf_sdm.h>
#include "nrf_sdh_ble.h"
#include "nrf_sdh_ant.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include <nrf_power.h>
#include <nrf_bootloader_info.h>

#include <ant_error.h>
#include <ant_search_config.h>
#include <ant_key_manager.h>
#include <ant_hrm.h>
#include <ant_parameters.h>
#include <ant_interface.h>


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "configuration.h"

#define WAKEUP_BUTTON_ID                0                                            /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID       1                                            /**< Button used for deleting all bonded centrals during startup. */

#define DEVICE_NAME                     "SturmFront"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "hannesweisbach"                        /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                40                                           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION                0                                           /**< The advertising duration in units of 10ms; 0 for indefinite */

#define APP_BLE_CONN_CFG_TAG            1                                            /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                            /**< Whether or not to include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device */

#define CONN_INTERVAL_BASE              80                                           /**< Definition of 100 ms, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                          /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (CONN_INTERVAL_BASE / 2)                     /**< Minimum acceptable connection interval (50 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (CONN_INTERVAL_BASE)                         /**< Maximum acceptable connection interval (100 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ANT_HRMRX_TRANS_TYPE            0                                            /**< Transmission Type. */
#define ANTPLUS_NETWORK_NUMBER          0                                            /**< Network number. */


static volatile uint16_t                m_conn_handle = BLE_CONN_HANDLE_INVALID;     /**< Handle of the current connection. */
static uint8_t                          m_adv_handle;                                /**< Advertising handle. */
BLE_HRS_DEF(m_hrs);                                                                  /**< Heart rate service instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                              /**< Context for the Queued Write module.*/
NRF_BLE_GATT_DEF(m_gatt);                                                            /**< GATT module instance. */

static bool entering_dfu{false};

static ble_sfc_t m_sfc;
NRF_SDH_BLE_OBSERVER(m_sfc_obs, BLE_SFC_BLE_OBSERVER_PRIO, ble_sfc_on_ble_evt,
                     &m_sfc);

NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
    .handler =
        [](nrf_sdh_state_evt_t state, void *) {
          if (state == NRF_SDH_EVT_STATE_DISABLED) {
            // Softdevice was disabled before going into reset. Inform
            // bootloader to skip CRC on next boot.
            nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

            // Go to system off.
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
          }
        },
};

class ZC {
#if 0
  // simulated ZC pin input
  static const int AC_LINE_ZC_PIN = 23;
#else
  // actual ZC pin input
  static const int AC_LINE_ZC_PIN = 22;
#endif

public:
  void setup() {
    nrfx_gpiote_in_config_t zc = {.sense = NRF_GPIOTE_POLARITY_HITOLO,
                                  .pull = NRF_GPIO_PIN_PULLUP,
                                  .is_watcher = false,
                                  .hi_accuracy = true,
                                  .skip_gpio_setup = false};
    APP_ERROR_CHECK(nrfx_gpiote_in_init(AC_LINE_ZC_PIN, &zc, NULL));
  }

  void begin() { nrfx_gpiote_in_event_enable(AC_LINE_ZC_PIN, false); }

  auto get_zc_eep() const {
    return nrfx_gpiote_in_event_addr_get(AC_LINE_ZC_PIN);
  }
} zc;

class FrequencyCounter {
  nrfx_timer_t zc_counter = NRFX_TIMER_INSTANCE(3);
  nrfx_timer_t timer = NRFX_TIMER_INSTANCE(4);

  // measure ~1s
  static const int measurement_periods = 110;

  uint32_t measured_line_period_us = 0;
  float measured_line_freq_hz = 0.0;

public:
  void init(uint32_t eep) {
    nrfx_timer_config_t counter_cfg{.frequency = NRF_TIMER_FREQ_1MHz,
                                    .mode = NRF_TIMER_MODE_COUNTER,
                                    .bit_width = NRF_TIMER_BIT_WIDTH_8,
                                    .interrupt_priority =
                                        NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
                                    .p_context = this};
    APP_ERROR_CHECK(nrfx_timer_init(
        &zc_counter, &counter_cfg, [](nrf_timer_event_t evt, void *context) {
          auto ctr = reinterpret_cast<FrequencyCounter *>(context);
          ctr->measured_line_period_us =
              nrfx_timer_capture_get(&ctr->timer, NRF_TIMER_CC_CHANNEL0) * 2 /
              (ctr->measurement_periods);
          ctr->measured_line_freq_hz =
              1000.0f * 1000.0f /
              static_cast<float>(ctr->measured_line_period_us);

          if (0) {
            const uint32_t ticks =
                nrfx_timer_capture_get(&ctr->timer, NRF_TIMER_CC_CHANNEL0);

            NRF_LOG_INFO("Ticks: %u %u / " NRF_LOG_FLOAT_MARKER, ticks,
                         ctr->line_period_us(),
                         NRF_LOG_FLOAT(ctr->line_freq()));
          }
        }));

    static_assert(measurement_periods <= UINT8_MAX,
                  "increase bit-width of counter");
    nrfx_timer_extended_compare(&zc_counter, NRF_TIMER_CC_CHANNEL0,
                                measurement_periods,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    APP_ERROR_CHECK(nrfx_timer_init(&timer, &timer_cfg, NULL));

    /* channel from ZC input pin to counter */
    nrf_ppi_channel_t zc_events;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&zc_events));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(
        zc_events, eep,
        nrfx_timer_task_address_get(&zc_counter, NRF_TIMER_TASK_COUNT)));

    /* channel from counter compare to timer stop */
    nrf_ppi_channel_t timer_stop;
    APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&timer_stop));
    APP_ERROR_CHECK(nrfx_ppi_channel_assign(
        timer_stop,
        nrfx_timer_event_address_get(&zc_counter, NRF_TIMER_EVENT_COMPARE0),
        nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_CAPTURE0)));
    /* also clear timer on counter compare */
    APP_ERROR_CHECK(nrfx_ppi_channel_fork_assign(
        timer_stop, nrfx_timer_task_address_get(&timer, NRF_TIMER_TASK_CLEAR)));

    APP_ERROR_CHECK(nrfx_ppi_channel_enable(zc_events));
    APP_ERROR_CHECK(nrfx_ppi_channel_enable(timer_stop));

    nrfx_timer_enable(&zc_counter);
    nrfx_timer_enable(&timer);
  }

  uint32_t line_period_us() { return measured_line_period_us; }
  float line_freq() { return measured_line_freq_hz; }
} line_freq_detect;

class Fan {
  class SSR {
    // SSR pin output
    static const int SSR_PIN = 17;
    static const unsigned PRE_ZC_US = 550;

    ZC zc;

    nrfx_timer_t delay_timer = NRFX_TIMER_INSTANCE(2);
    bool enabled_ = false;

    unsigned map_duty_to_us(unsigned pct) {
      const unsigned half_cycle_us = line_freq_detect.line_period_us() / 2;
      const unsigned hold_off_pct =
          (half_cycle_us * 100) - (half_cycle_us * pct);

      return hold_off_pct / 100;
    }

    void ssr_configure_ticks(const uint32_t ticks) {
      nrfx_timer_compare(&delay_timer, NRF_TIMER_CC_CHANNEL0, ticks, false);
    }

  public:
    void setup(const uint32_t zc_eep) {
      nrfx_gpiote_out_config_t ssr_config = {
          .action =
              static_cast<nrf_gpiote_polarity_t>(GPIOTE_CONFIG_POLARITY_None),
          .init_state = NRF_GPIOTE_INITIAL_VALUE_LOW,
          .task_pin = true,
      };
      APP_ERROR_CHECK(nrfx_gpiote_out_init(SSR_PIN, &ssr_config));

      nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
      APP_ERROR_CHECK(nrfx_timer_init(&delay_timer, &timer_cfg, NULL));

      nrfx_timer_compare(&delay_timer, NRF_TIMER_CC_CHANNEL0, 0, false);

      // ZC turns SSR off
      // ZC clears delay timer
      // ZC starts delay timer
      // delay timer CC triggers SSR
      // delay timer CC stops delay timer

      nrf_ppi_channel_t timer_ch;
      APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&timer_ch));
      APP_ERROR_CHECK(nrfx_ppi_channel_assign(
          timer_ch, zc_eep,
          nrfx_timer_task_address_get(&delay_timer, NRF_TIMER_TASK_CLEAR)));
      APP_ERROR_CHECK(nrfx_ppi_channel_enable(timer_ch));

      nrf_ppi_channel_t ssr_clr_ch;
      APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ssr_clr_ch));
      APP_ERROR_CHECK(nrfx_ppi_channel_assign(
          ssr_clr_ch, zc_eep, nrfx_gpiote_clr_task_addr_get(SSR_PIN)));
      APP_ERROR_CHECK(nrfx_ppi_channel_enable(ssr_clr_ch));

      nrf_ppi_channel_t ssr_set_ch;
      APP_ERROR_CHECK(nrfx_ppi_channel_alloc(&ssr_set_ch));
      APP_ERROR_CHECK(nrfx_ppi_channel_assign(
          ssr_set_ch,
          nrfx_timer_event_address_get(&delay_timer, NRF_TIMER_EVENT_COMPARE0),
          nrfx_gpiote_set_task_addr_get(SSR_PIN)));
      APP_ERROR_CHECK(nrfx_ppi_channel_enable(ssr_set_ch));

      nrfx_timer_enable(&delay_timer);
    }

    void enable() {
      if (enabled_) {
        return;
      }

      NRF_LOG_INFO("SSR Enabled");

      nrfx_gpiote_out_task_enable(SSR_PIN);

      enabled_ = true;
      // m_sfc.update_ssr_state(enabled_);
    }

    void disable() {
      if (disabled()) {
        return;
      }

      NRF_LOG_INFO("Disabling SSR");

      nrfx_gpiote_out_task_disable(SSR_PIN);
      nrfx_gpiote_out_clear(SSR_PIN);

      enabled_ = false;
      // m_sfc.update_ssr_state(enabled_);
    }

    void set_percent(const uint8_t duty, bool do_enable = true) {
      if (do_enable && disabled()) {
        enable();
      }

      const unsigned us = map_duty_to_us(duty);
      const unsigned ticks =
          nrfx_timer_us_to_ticks(&delay_timer, us + PRE_ZC_US);

      if (duty > 0 && duty < 100) {
        NRF_LOG_RAW_INFO("/r:%u/%d us\n", duty, ticks);
        ssr_configure_ticks(ticks);
        if (enabled()) {
          nrfx_gpiote_out_task_enable(SSR_PIN);
        }
      } else if (duty <= 0 || disabled()) {
        nrfx_gpiote_out_task_disable(SSR_PIN);
        nrfx_gpiote_out_clear(SSR_PIN);
      } else if (duty >= 100) {
        nrfx_gpiote_out_task_disable(SSR_PIN);
        nrfx_gpiote_out_set(SSR_PIN);
      }
    }

    bool enabled() const { return enabled_; }
    bool disabled() const { return !enabled_; }

  } ssr;

  unsigned hr_ = 0;
  unsigned temperature_ = 0;
  enum class State { Off, Spinup, On } state = State::Off;
  static const constexpr int spinupDelay = 5;
  int spinCount = 0;

  unsigned remap_min_duty(const unsigned duty, enum State state) {
    const auto min_duty = (state == State::On) ? m_sfc.setting(MIN_DUTY_WARM)
                                               : m_sfc.setting(MIN_DUTY_COLD);
    return duty * (100 - min_duty) / 100 + min_duty;
  }

public:
  void setup(const unsigned eep) { ssr.setup(eep); }

  void set_duty(const uint8_t duty, const bool debug = false) {
    if (debug) {
      // spin up again, when exiting debug mode
      state = State::Off;
      ssr.set_percent(duty);
      return;
    }

    const bool on = duty > 0;

    NRF_LOG_RAW_INFO("/s:%d", state);

    switch (state) {
    case State::Off:
      if (on) {
        state = State::Spinup;
        spinCount = 0;
      }
      break;
    case State::Spinup:
      if (spinCount >= spinupDelay) {
        state = State::On;
      }
      ++spinCount;
      break;
    case State::On:
      break;
    }

    if (state != State::Off && !on) {
      state = State::Off;
    }

    if (state == State::Off) {
      ssr.disable();
      NRF_LOG_RAW_INFO("\n");
    } else {
      ssr.set_percent(remap_min_duty(duty, state));
    }
  }

  void hr(const unsigned hr) { hr_ = hr; }
  void temperature(const unsigned temp) { temperature_ = temp; }

  void on() { ssr.set_percent(100); }
  void off() {
    ssr.set_percent(0, false);
    ssr.disable();
    state = State::Off;
  }
} fan;

#define MY_APP_TIMER_DEF(timer_id)                                             \
  NRF_LOG_INSTANCE_REGISTER(                                                   \
      APP_TIMER_LOG_NAME, timer_id, APP_TIMER_CONFIG_INFO_COLOR,               \
      APP_TIMER_CONFIG_DEBUG_COLOR, APP_TIMER_CONFIG_INITIAL_LOG_LEVEL,        \
      APP_TIMER_CONFIG_LOG_ENABLED ? APP_TIMER_CONFIG_LOG_LEVEL                \
                                   : NRF_LOG_SEVERITY_NONE);                   \
  app_timer_t CONCAT_2(timer_id, _data) = {                                    \
      .end_val = APP_TIMER_IDLE_VAL,                                           \
      NRF_LOG_INSTANCE_PTR_INIT(p_log, APP_TIMER_LOG_NAME, timer_id)};         \
  const app_timer_id_t timer_id = &CONCAT_2(timer_id, _data)

class IO {
  /* LED */

  bool ant_connected_ = false;
  bool needs_update_ = true;
  bsp_indication_t ble_mode_ = BSP_INDICATE_IDLE;

  static const uint32_t dim_ms =
      UINT8_MAX * 8 * 1000 /
      (APP_TIMER_CLOCK_FREQ / (APP_TIMER_CONFIG_RTC_FREQUENCY + 1));

  void init_ant_led() {
    led_sb_init_params_t led_sb_init_params =
        LED_SB_INIT_DEFAULT_PARAMS(LEDS_MASK);
    led_sb_init_params.off_time_ticks = APP_TIMER_TICKS(500 - dim_ms);
    led_sb_init_params.on_time_ticks = APP_TIMER_TICKS(500 - dim_ms);
    led_sb_init_params.duty_cycle_max = 240;
    led_sb_init_params.duty_cycle_min = 0;
    led_sb_init_params.duty_cycle_step = 240 / 8;

    APP_ERROR_CHECK(led_softblink_init(&led_sb_init_params));
  }

  void indicate_ble() {
    NRF_LOG_INFO("Set BSP indication: %x", ble_mode_);
    APP_ERROR_CHECK(led_softblink_stop());
    APP_ERROR_CHECK(bsp_indication_set(ble_mode_));
  }

  /* Button */
#if 1
  static const constexpr bsp_event_t BSP_EVENT_KEY_0_RELEASE =
      BSP_EVENT_DISCONNECT;
#else
  // C++ does not allow for new enum values to be invented.
  static const constexpr bsp_event_t BSP_EVENT_KEY_0_RELEASE =
      static_cast<bsp_event_t>(BSP_EVENT_KEY_LAST + 1);
#endif

  static const constexpr uint32_t BTN_ID = 0u;
  bool btn_timer_running = false;
  bool long_press_detected = false;
  MY_APP_TIMER_DEF(long_btn_press_timer);
  inline static IO *instance = nullptr;

  std::function<void()> long_press_handler;
  std::function<void()> short_press_handler;

  void start_timer() {
    if (btn_timer_running) {
      return;
    }
    APP_ERROR_CHECK(
        app_timer_start(long_btn_press_timer, APP_TIMER_TICKS(3500), this));
    btn_timer_running = true;
  }

  void stop_timer() {
    APP_ERROR_CHECK(app_timer_stop(long_btn_press_timer));
    btn_timer_running = false;
  }

  void timer_timeout() {
    long_press_detected = true;
    long_button_press();
  }

  void bsp_button_evt_handler(bsp_event_t event) {
    NRF_LOG_INFO("BSP event: %u", event);
    switch (event) {
    case BSP_EVENT_KEY_0:
      start_timer();
      break;
    case BSP_EVENT_KEY_0_RELEASE:
      stop_timer();
      if (!long_press_detected) {
        short_button_press();
      } else {
        long_press_detected = false;
      }
     break;
    default:
      break;
    }
  }

  void long_button_press() {
    if (long_press_handler) {
      long_press_handler();
    }
  }

  void short_button_press() {
    if (short_press_handler) {
      short_press_handler();
    }
  }

public:
  IO() = default;
  IO(IO &) = delete;
  IO(IO &&) = delete;

  template <typename LONG_F, typename SHORT_F>
  void init(SHORT_F &&short_press_hdlr, LONG_F &&long_press_hdlr) {
    instance = this;
    short_press_handler = std::move(short_press_hdlr);
    long_press_handler = std::move(long_press_hdlr);
    APP_ERROR_CHECK(app_timer_create(&long_btn_press_timer,
                                     APP_TIMER_MODE_SINGLE_SHOT, [](void *ctx) {
                                       IO *io = static_cast<IO *>(ctx);
                                       io->timer_timeout();
                                     }));
    APP_ERROR_CHECK(
        bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, [](bsp_event_t event) {
          instance->bsp_button_evt_handler(event);
        }));
    APP_ERROR_CHECK(bsp_event_to_button_action_assign(
        BTN_ID, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_KEY_0_RELEASE));
    init_ant_led();
  }

  void loop(const bool ant_connected) {
    const bool update = ant_connected != ant_connected_;
    ant_connected_ = ant_connected;

    if (update || needs_update_) {
      NRF_LOG_INFO("Switching LED to %s", ((ant_connected) ? "ANT" : "BLE"));

      if (ant_connected) {
        APP_ERROR_CHECK(bsp_indication_set(BSP_INDICATE_IDLE));
        APP_ERROR_CHECK(led_softblink_start(BSP_LED_0_MASK));
      } else {
        indicate_ble();
      }

      if (needs_update_) {
        needs_update_ = false;
      }
    }
  }

  void set_ble_mode(const bsp_indication_t ble_mode) {
    needs_update_ = ble_mode != ble_mode_;
    ble_mode_ = ble_mode;
  }

  void set_ant_bpm(const uint32_t bpm) {
    if (bpm == 0) {
      return;
    }
    const uint32_t ms_per_beat = (60.0f * 1000.0f) / bpm;
    const uint32_t dim_ticks = APP_TIMER_TICKS(dim_ms);
    uint32_t ticks = APP_TIMER_TICKS(ms_per_beat) / 2 - dim_ticks;
    //NRF_LOG_INFO("LED BPM: %u, ms: %u ticks: %u", bpm, ms_per_beat, ticks);
    led_softblink_off_time_set(ticks);
    led_softblink_on_time_set(ticks);
  }

  void on() {
    APP_ERROR_CHECK(led_softblink_stop());
    APP_ERROR_CHECK(bsp_indication_set(BSP_INDICATE_CONNECTED));
    needs_update_ = true;
  }
  void off() {
    APP_ERROR_CHECK(led_softblink_stop());
    APP_ERROR_CHECK(bsp_indication_set(BSP_INDICATE_IDLE));
    needs_update_ = true;
  }
} io;

class ANTHRM {
public:
  static const constexpr int ANT_HRMRX_DEVICE_NUMBER_ANY = 0x0000;

private:
  static const constexpr int ANT_HRMRX_ANT_CHANNEL = 0;

  ant_hrm_profile_t m_ant_hrm;
  enum class State { Normal, Reconfigure } state = State::Normal;
  uint16_t new_device_number_ = ANT_HRMRX_DEVICE_NUMBER_ANY;
  uint8_t old_channel_state = 0xaa; // only for debug output.

  uint8_t channel_state() {
    uint8_t channel_status = 0xaa;
    APP_ERROR_CHECK(
        sd_ant_channel_status_get(m_ant_hrm.channel_number, &channel_status));
    if (channel_status != old_channel_state) {
      old_channel_state = channel_status;
      NRF_LOG_INFO("Channel state: %hhx", channel_status);
    }
    return channel_status;
  }

  bool channel_open() {
    const uint8_t state = channel_state();
    return (state == STATUS_SEARCHING_CHANNEL) ||
           (state == STATUS_TRACKING_CHANNEL);
  }

  void reassign_channel() {
    NRF_LOG_INFO("Unassign channel");
    ret_code_t err_code = sd_ant_channel_unassign(m_ant_hrm.channel_number);
    NRF_LOG_INFO("Unassign: %x", err_code);
    // Assign new device number
    setup(new_device_number_);
  }

  public:
    ANTHRM() = default;

    void setup(uint16_t device_number = ANT_HRMRX_DEVICE_NUMBER_ANY) {
      // Initialize ANT+ HRM receive channel.
      NRF_LOG_INFO("ANT Setup");
      HRM_DISP_CHANNEL_CONFIG_DEF(m_ant_hrm, ANT_HRMRX_ANT_CHANNEL,
                                  ANT_HRMRX_TRANS_TYPE, device_number,
                                  ANTPLUS_NETWORK_NUMBER, HRM_MSG_PERIOD_4Hz);
      APP_ERROR_CHECK(
          ant_hrm_disp_init(&m_ant_hrm, HRM_DISP_CHANNEL_CONFIG(m_ant_hrm),
                            [](ant_hrm_profile_t *, ant_hrm_evt_t) {}));
    }

    void begin() {
      const ret_code_t err = ant_hrm_disp_open(&m_ant_hrm);
      NRF_LOG_INFO("Opening ANT channel %d", err);
    }

    bool end() {
      const bool ch_open = channel_open();
      if (ch_open) {
        APP_ERROR_CHECK(sd_ant_channel_close(m_ant_hrm.channel_number));
        NRF_LOG_INFO("Closed.");
      } else {
        NRF_LOG_INFO("ANT channel already closed.");
      }

      return ch_open;
    }

    void reconfigure(uint16_t device_number = ANT_HRMRX_DEVICE_NUMBER_ANY) {
      // We need to wait for the CHANNEL_CLOSED event, before unassigning the
      // channel and re-assigning with the new device number. This is done via
      // the state change in ant_evt_handler(). Store new device number until
      // then.
      // If channel was not opened, we can reassign the device number now.
      new_device_number_ = device_number;
      state = State::Reconfigure;
      const bool channel_was_open = end();
      if (!channel_was_open) {
        reassign_channel();
      }
    }

    uint32_t hr() const { return m_ant_hrm.page_0.computed_heart_rate; }
    bool connected() { return channel_state() == STATUS_TRACKING_CHANNEL; }

    void ant_evt_handler(ant_evt_t * event) {
      ant_hrm_disp_evt_handler(event, &m_ant_hrm);

      switch (event->event) {
      case EVENT_CHANNEL_CLOSED:
        NRF_LOG_INFO("ANT channel closed.");
        if (state == State::Reconfigure) {
          state = State::Normal;
          channel_state();
          reassign_channel();
        }
        begin();
        // probably don't want to do that.
        // device_number_ = ANT_HRMRX_DEVICE_NUMBER_ANY;
        break;
      case EVENT_RX:
        if (m_sfc.setting(ANT_DEVICE) == ANT_HRMRX_DEVICE_NUMBER_ANY) {
        const uint16_t devid = uint16_decode(event->message.ANT_MESSAGE_aucExtData);
          NRF_LOG_INFO("Setting ANT dev id to: 0x%04x", devid);
          m_sfc.setting(ANT_DEVICE, devid);
        }
        break;
      default:
        NRF_LOG_INFO("ANT event: %x", event->event);
      }
    }
} hrm;

NRF_SDH_ANT_OBSERVER(
    m_ant_hrm_observer, ANT_HRM_ANT_OBSERVER_PRIO,
    [](ant_evt_t *event, void *context) {
      ANTHRM *hrm = static_cast<ANTHRM *>(context);
      hrm->ant_evt_handler(event);
    },
    &hrm);

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
  switch (event) {
  case NRF_PWR_MGMT_EVT_PREPARE_DFU:
    // YOUR_JOB: Get ready to reset into DFU mode
    //
    // If you aren't finished with any ongoing tasks, return "false" to
    // signal to the system that reset is impossible at this stage.
    //
    // Here is an example using a variable to delay resetting the device.
    //
    // if (!m_ready_for_reset)
    // {
    //      return false;
    // }
    // else
    // TODO wait for flash
    {
      fan.off();
      NRF_LOG_INFO("Power management wants to reset to DFU mode.");
      APP_ERROR_CHECK(sd_softdevice_disable());
      APP_ERROR_CHECK(app_timer_stop_all());
      NRF_LOG_FLUSH();
    }
    break;

  default:
    // YOUR_JOB: Implement any of the other events available from the power
    // management module:
    //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
    //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
    //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
    return true;
  }

  NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
  error_info_t *error_info = (error_info_t *)info;
  NRF_LOG_INFO("ERROR: %x Line %d (%s)", error_info->err_code,
               error_info->line_num, error_info->p_file_name);

  NRF_LOG_FINAL_FLUSH();
  for (;;) {
  }
}

class App {
  MY_APP_TIMER_DEF(timer_tick);
  unsigned temperature = 0;

  void update_temperature() {
    int32_t temp;
    const ret_code_t err = sd_temp_get(&temp);
    if (err == NRF_SUCCESS) {
      temperature = temp / 4;
      m_sfc.update_temperature(temperature);
    }
  }

  enum class Mode { Auto, On, Off } mode_ = Mode::Auto;

public:
  void init() {
    APP_ERROR_CHECK(
        app_timer_create(&timer_tick, APP_TIMER_MODE_REPEATED, [](void *ctx) {
          App *app = static_cast<App *>(ctx);
          if (app != nullptr) {
            app->tick();
          }
        }));
    APP_ERROR_CHECK(app_timer_start(timer_tick, APP_TIMER_TICKS(1000), this));
  }

  /* main app tick loop */
  void tick() {
    m_sfc.update_line_frequency(line_freq_detect.line_freq());

    update_temperature();

    const uint8_t debug_duty = m_sfc.setting(DEBUG_DUTY);
    const bool debug = debug_duty > 0;
    if (debug) {
      fan.set_duty(debug_duty, debug);
      return;
    }

    const uint8_t debug_hr = m_sfc.setting(DEBUG_HR);
    if (debug_hr) {
      const auto duty_ = m_sfc.map(debug_hr, temperature);
      fan.set_duty(duty_);
      io.set_ant_bpm(debug_hr);
      io.loop(true);
      return;
    } 

    switch (mode_) {
    case Mode::Auto: {
      if (hrm.connected()) {
        const uint32_t hr = hrm.hr();
        // switch LED over to ANT to show HR
        io.set_ant_bpm(hr);
        ble_hrs_heart_rate_measurement_send(&m_hrs, hr);
        const auto duty = m_sfc.map(hr, temperature);
        fan.set_duty(duty);
      } else {
        fan.off();
      }
      io.loop(hrm.connected());
      break;}
    case Mode::On:
      fan.on();
      io.on();
      break;
    case Mode::Off:
      fan.off();
      io.off();
      break;
    }
  }

  void toggleMode() {
    switch (mode_) {
    case Mode::Auto:
      mode_ = Mode::On;
      break;
    case Mode::On:
      mode_ = Mode::Off;
      break;
    case Mode::Off:
      mode_ = Mode::Auto;
      break;
    }
    NRF_LOG_INFO("Mode: %s", mode_ == Mode::On    ? "On"
                             : mode_ == Mode::Off ? "Off"
                                                  : "Auto");
  }
} app;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }

    io.set_ble_mode(BSP_INDICATE_ADVERTISING);
    io.loop(false);
}

/**@brief Attempt to both open the ant channel and start ble advertising.
*/
static void ant_and_adv_start(void)
{
    advertising_start();
    hrm.begin();
}

/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_CYCLING);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t             err_code;
    ble_advdata_t        advdata;
    ble_advdata_t srdata;
    ble_gap_adv_data_t   advdata_enc;
    ble_gap_adv_params_t adv_params;
    static uint8_t       advdata_buff[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    uint16_t             advdata_buff_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
#if 1
    static uint8_t       srdata_buff[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    uint16_t             srdata_buff_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
#endif
    uint8_t              flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    };


    ble_uuid_t sr_uuids[] = {m_sfc.get_service_uuid()};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = flags;
#if 0
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
#else
    advdata.uuids_more_available.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_more_available.p_uuids  = adv_uuids;
#endif
    
    memset(&srdata, 0, sizeof(srdata));

    srdata.uuids_complete.uuid_cnt = sizeof(sr_uuids) / sizeof(sr_uuids[0]);
    srdata.uuids_complete.p_uuids  = sr_uuids;

    err_code = ble_advdata_encode(&advdata, advdata_buff, &advdata_buff_len);
    APP_ERROR_CHECK(err_code);

    (void)srdata_buff;
    (void)srdata_buff_len;
    err_code = ble_advdata_encode(&srdata, srdata_buff, &srdata_buff_len);
    APP_ERROR_CHECK(err_code);

    m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

    memset(&advdata_enc, 0, sizeof(advdata_enc));

    advdata_enc.adv_data.p_data = advdata_buff;
    advdata_enc.adv_data.len    = advdata_buff_len;

    advdata_enc.scan_rsp_data.p_data = srdata_buff;
    advdata_enc.scan_rsp_data.len    = srdata_buff_len;

    // Initialise advertising parameters (used when starting advertising).
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;
    adv_params.duration        = APP_ADV_DURATION;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &advdata_enc, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            // Prevent device from advertising on disconnect.
            entering_dfu = true;

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(
                [](const uint16_t conn_handle, void *) {
                  const ret_code_t err_code = sd_ble_gap_disconnect(
                      conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                  if (err_code != NRF_SUCCESS) {
                    NRF_LOG_WARNING("Failed to disconnect connection. "
                                    "Connection handle: %d Error: %d",
                                    conn_handle, err_code);
                  } else {
                    NRF_LOG_DEBUG("Disconnected connection handle %d",
                                  conn_handle);
                  }
                },
                NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}



/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Heart Rate and Device Information services.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_hrs_init_t     hrs_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize the Queued Write module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_OTHER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = false;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str,
                          const_cast<char *>(MANUFACTURER_NAME));
    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    err_code = ble_sfc_init(&m_sfc);
    APP_ERROR_CHECK(err_code);

    ble_dfu_buttonless_init_t dfus_init = {0};
    dfus_init.evt_handler = ble_dfu_evt_handler;

    APP_ERROR_CHECK(ble_dfu_buttonless_init(&dfus_init));
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = [](const uint32_t nrf_error) {
      APP_ERROR_HANDLER(nrf_error);
    };

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 * @param[in] p_context  Context.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            io.set_ble_mode(BSP_INDICATE_CONNECTED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            io.set_ble_mode(BSP_INDICATE_IDLE);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Need to close the ANT channel to make it safe to write bonding information to flash
            hrm.end();
            if (!entering_dfu) {
              advertising_start();
            }
            // Note: Bonding information will be stored, advertising will be restarted and the
            //       ANT channel will be reopened when ANT event CHANNEL_CLOSED is received.
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .tx_phys = BLE_GAP_PHY_AUTO,
                .rx_phys = BLE_GAP_PHY_AUTO,
            };

            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

#ifndef BONDING_ENABLE
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;
#endif // BONDING_ENABLE

        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason ==
                BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
            {
                io.set_ble_mode(BSP_INDICATE_IDLE);
                NRF_LOG_INFO("System off.");
                NRF_LOG_FLUSH();
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

#ifndef BONDING_ENABLE
            case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle,
                                                     NULL,
                                                     0,
                                                     BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
                APP_ERROR_CHECK(err_code);
                break;
#endif // BONDING_ENABLE

        default:
            // No implementation needed.
            break;
    }
}

#ifdef BONDING_ENABLE
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}
#endif // BONDING_ENABLE


/**@brief BLE + ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the stack event interrupt.
 */
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("RAM start: %x", ram_start);

#if 1
    ble_cfg_t ble_cfg{0};
    ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = 2000;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
#endif

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

template <typename Pred> static void idle_loop(Pred &&p) {
  // Enter main loop.
  for (; p();) {
    if (NRF_LOG_PROCESS() == false) {
      nrf_pwr_mgmt_run();
    }
  }
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Initialize peripherals
    timers_init();


    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    ble_sfc_init_static(&m_sfc);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    softdevice_setup();

    // FDS initialization
    {
      static std::atomic_bool fds_initialized{false};

      fds_register([](fds_evt_t const *p_evt) {
        switch (p_evt->id) {
        case FDS_EVT_INIT:
          if (p_evt->result == NRF_SUCCESS) {
            fds_initialized = true;
          }
          break;
        case FDS_EVT_GC:
          NRF_LOG_INFO("FDS GC done.");
          break;
        default:
          break;
        }
      });

      APP_ERROR_CHECK(fds_init());
      idle_loop([] { return !fds_initialized.load(); });
      APP_ERROR_CHECK(fds_gc());
    }

    io.init(
        []() {
          app.toggleMode();
          app.tick();
        },
        []() {
          m_sfc.setting(ANT_DEVICE, ANTHRM::ANT_HRMRX_DEVICE_NUMBER_ANY);
          hrm.reconfigure();
        });

    // Initialize Bluetooth stack parameters.
    gatt_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    hrm.setup(m_sfc.setting(ANT_DEVICE));

#ifdef BONDING_ENABLE
    bool erase_bonds = bsp_button_is_pressed(BOND_DELETE_ALL_BUTTON_ID);
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        NRF_LOG_INFO("Bonds erased!");
    }
#endif // BONDING_ENABLE

    zc.setup();

    line_freq_detect.init(zc.get_zc_eep());
    fan.setup(zc.get_zc_eep());
    //ac_line_simulation_setup(&m_sfc);


    NRF_LOG_INFO("SF ready.");
    ant_and_adv_start();

    zc.begin();

    // Start application timer tick.
    app.init();

    idle_loop([] { return true; });
}

/**
 * @}
 */
