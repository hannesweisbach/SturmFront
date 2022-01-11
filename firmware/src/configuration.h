#pragma once

#include <stdint.h>

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
                   
#define BLE_SFC_BLE_OBSERVER_PRIO 1

enum ConfigurationSetting {
  ANT_DEVICE = 0,
  MIN_DUTY_COLD,
  MIN_DUTY_WARM,
  MAX_DUTY,
  DEBUG_HR,
  DEBUG_DUTY,
};

typedef struct ble_sfc_s {
  uint16_t service_handle;
  uint16_t conn_handle_ = BLE_CONN_HANDLE_INVALID;
  uint8_t uuid_type = 0;

  ble_gatts_char_handles_t reset;
  ble_gatts_char_handles_t line_frequency;
  ble_gatts_char_handles_t line_frequency_sim;
  ble_gatts_char_handles_t line_sim;
  float line_freq_sim_value;
  bool line_freq_sim_value_updated;
  uint8_t line_sim_value;
  bool line_sim_updated;

  ble_gatts_char_handles_t temperature_char;
  uint8_t temperature;

private:
  void send_ble_notification(uint8_t const *const ptr, const uint16_t len,
                             const uint16_t value_handle);

public:
  ble_uuid_t get_service_uuid() const;

  template <typename T>
  void send_ble_notification(T &&value, const uint16_t value_handle) {
    static_assert(sizeof(T) < (1 << 16), "Object too big.");

    send_ble_notification(reinterpret_cast<uint8_t const *>(&value), sizeof(T),
                          value_handle);
  }

  void update_temperature(const uint8_t t) {
    send_ble_notification(t, temperature_char.value_handle);
  }

  void update_line_frequency(const float hz) {
    send_ble_notification(hz, line_frequency.value_handle);
  }

  bool ble_connected() const { return conn_handle_ != BLE_CONN_HANDLE_INVALID; }
  bool ble_disconnected() const {
    return conn_handle_ == BLE_CONN_HANDLE_INVALID;
  }
  void ble_disconnect() { conn_handle_ = BLE_CONN_HANDLE_INVALID; }
  void ble_connect(const uint16_t conn_handle) { conn_handle_ = conn_handle; }

  unsigned setting(const enum ConfigurationSetting setting) const;
  void setting(const enum ConfigurationSetting setting,
               const unsigned value) const;

  unsigned map(const unsigned temp, const unsigned bpm) const;
} ble_sfc_t;

#ifdef __cplusplus
extern "C" {
#endif


void ble_sfc_init_static(ble_sfc_t * handle);
ret_code_t ble_sfc_init(ble_sfc_t * handle);

void ble_sfc_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#ifdef __cplusplus
}
#endif
