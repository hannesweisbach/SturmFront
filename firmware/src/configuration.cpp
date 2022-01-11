#include <array>
#include <cmath>
#include <tuple>
#include <functional>
#include <vector>

#include <sdk_common.h>

#include <fds.h>

#include <nrf_log.h>
#include <nrf_log_ctrl.h>

#include "configuration.h"

#define SF_UUID_BASE    {{0x26, 0xF1, 0x85, 0xD0, 0x0C, 0x24, 0x4E, 0xD3, 0x82, 0x86, 0x0B, 0x2E, 0x9B, 0x5D, 0xA2, 0x03}}
#define SF_UUID_SERVICE   0xD34E
#define SF_UUID_LINE      0xD34F
#define SF_UUID_LINE_SIM  0xD350
#define SF_UUID_SIM       0xD351

#define SF_UUID_FIRST         0xD360

#define SF_UUID_ANT_DEVICE    SF_UUID_FIRST
#define SF_UUID_MIN_DUTY_COLD 0xD361
#define SF_UUID_MIN_DUTY_WARM 0xD362
#define SF_UUID_MAX_DUTY      0xD363
#define SF_UUID_MAP           0xD364
#define SF_UUID_DBG_HR        0xD365
#define SF_UUID_DBG_DUTY      0xD366

#define SF_UUID_LAST          0xD366

#define SF_UUID_HANDLER_FIRST 0xD370
#define SF_UUID_MAP_SINGLE    SF_UUID_HANDLER_FIRST
#define SF_UUID_CONF_RESET    0xD371 
#define SF_UUID_SSR_OVR       0xD372

#define SF_UUID_HANDLER_LAST  0xD371

#define SF_UUID_TEMP      0xD380


using word_t = uint32_t;
static const constexpr auto word_size = sizeof(word_t);

template <typename T> static constexpr size_t size_in_words(T &&) {
  return (sizeof(T) + 3) / sizeof(word_t);
}

using gatt_unit_t = uint16_t;

class PersistentConfiguration {
  friend class ble_sfc_s;

  static const constexpr unsigned minTemp = 10;
  static const constexpr unsigned maxTemp = 30;
  static const constexpr unsigned tempStep = 5;
  static const constexpr unsigned tempPoints =
      ((maxTemp - minTemp) / tempStep) + 1;
  static const constexpr unsigned minHr = 105;
  static const constexpr unsigned maxHr = 145;
  static const constexpr unsigned hrStep = 5;
  static const constexpr unsigned hrPoints = ((maxHr - minHr) / hrStep) + 1;

  static_assert(maxTemp == minTemp + (tempPoints - 1) * tempStep,
                "Temperature range not evenly disible by tempStep.");
  static_assert(maxHr == minHr + (hrPoints - 1) * hrStep,
                "HR range not evenly disible by hrStep.");

  auto mapValueToIndices(const unsigned value, const unsigned min,
                         const unsigned step, const int points) {

    const auto offset = static_cast<float>(value) - min;
    const auto floatIdx = offset / step;
    const auto lowIdx =
        std::clamp(static_cast<int>(std::floor(floatIdx)), 0, points - 1);
    const auto highIdx =
        std::clamp(static_cast<int>(std::ceil(floatIdx)), 0, points - 1);
    return std::make_pair(lowIdx, highIdx);
  }

  uint8_t map(uint8_t hr, uint8_t temp) {
    int lowTempIdx, highTempIdx;
    std::tie(lowTempIdx, highTempIdx) =
        mapValueToIndices(temp, minTemp, tempStep, tempPoints);

    int lowHrIdx, highHrIdx;
    std::tie(lowHrIdx, highHrIdx) =
        mapValueToIndices(hr, minHr, hrStep, hrPoints);

    const float hrLerpFactor =
        (hr < minHr) ? 0.0f
                     : std::clamp((hr - minHr - lowHrIdx * hrStep) /
                                      static_cast<float>(hrStep),
                                  0.0f, 1.0f);
    const float tempLerpFactor =
        (temp < minTemp) ? 0.0f
                         : std::clamp((temp - minTemp - lowTempIdx * tempStep) /
                                          static_cast<float>(tempStep),
                                      0.0f, 1.0f);
    const auto minTempLerp =
        std::lerp(settings.at(lowTempIdx, lowHrIdx),
                  settings.at(lowTempIdx, highHrIdx), hrLerpFactor);
    const auto maxTempLerp =
        std::lerp(settings.at(highTempIdx, lowHrIdx),
                  settings.at(highTempIdx, highHrIdx), hrLerpFactor);

    const auto interpolated = static_cast<unsigned>(
        std::lerp(minTempLerp, maxTempLerp, tempLerpFactor));

    if (interpolated < 0 || interpolated > 100) {
      NRF_LOG_INFO("%d bpm, %d°C mapped to %d% DC", hr, temp, interpolated);
    }

    NRF_LOG_RAW_INFO("BPM: %d [%u:%u," NRF_LOG_FLOAT_MARKER "], ", hr, lowHrIdx,
                     highHrIdx, NRF_LOG_FLOAT(hrLerpFactor));
    NRF_LOG_RAW_INFO("Temp %u°C [%u:%u," NRF_LOG_FLOAT_MARKER "] ", temp,
                     lowTempIdx, highTempIdx, NRF_LOG_FLOAT(tempLerpFactor));
    NRF_LOG_RAW_INFO("-> %u", interpolated);

    return std::clamp(interpolated, 0u, 100u);
  }

  struct alignas(sizeof(word_t)) config_t {
    uint16_t ant_device;
    uint8_t min_duty_cold;
    uint8_t min_duty_warm;
    uint8_t max_duty;
    uint8_t map[tempPoints * hrPoints];

    uint8_t at(const int tempIdx, const int hrIdx) {
      return map[tempIdx * hrPoints + hrIdx];
    }
  };

  static const constexpr config_t defaults{
      .ant_device = 0,
      .min_duty_cold = 30,
      .min_duty_warm = 27,
      .max_duty = 94,
      .map =
          /* 105 110 115 120 125 130 135 140, 145 */
      {
          0, 0,  1,  5,  10, 20, 35,  70,  80,  /* 10° */
          0, 0,  2,  5,  12, 40, 60,  80,  100, /* 15° */
          0, 1,  5,  12, 20, 42, 65,  85,  100, /* 20° */
          0, 5,  20, 30, 45, 65, 85,  100, 100, /* 25° */
          0, 10, 25, 45, 65, 85, 100, 100, 100, /* 30° */
      },
  };

  static const constexpr gatt_unit_t GATT_UNIT_NONE = 0x2700;
  static const constexpr gatt_unit_t GATT_UNIT_HZ = 0x2722;
  static const constexpr gatt_unit_t GATT_UNIT_TEMP_C = 0x272F;
  static const constexpr gatt_unit_t GATT_UNIT_PERCENT = 0x27AD;
  static const constexpr gatt_unit_t GATT_UNIT_BPM = 0x27AF;

  struct descriptor {
    uint32_t value = 0;
    uint8_t *ptr;
    uint16_t size;
    uint16_t uuid;
    ble_gatts_char_handles_t handle;
    gatt_unit_t unit;
    uint8_t format;
    bool persistent = true;
    char const *const description;
    using handler_t = std::function<void(const uint16_t,
                                         ble_gatts_evt_write_t const *const evt,
                                         struct descriptor &)>;
    handler_t handler;

    descriptor(descriptor &) = delete;
    descriptor &operator=(descriptor &) = delete;

    template <typename T = uint8_t>
    descriptor(T *p = nullptr, uint16_t uuid = 0, const char *desc = nullptr,
               const gatt_unit_t unit = GATT_UNIT_NONE,
               const uint16_t format = 0, uint16_t sz = sizeof(T),
               const bool persistent = true, const bool is_handler = false)
        : ptr(reinterpret_cast<uint8_t *>(p)), size(sz), uuid(uuid), unit(unit),
          format(format), persistent(persistent), description(desc) {
      handle.value_handle = uuid;
    }

    descriptor(handler_t &&handler, uint16_t uuid = 0,
               const char *desc = nullptr,
               const gatt_unit_t unit = GATT_UNIT_NONE,
               const uint16_t format = 0, uint16_t sz = 1)
        : ptr(nullptr), size(sz), uuid(uuid), unit(unit), format(format),
          persistent(false), description(desc), handler(std::move(handler)) {
      handle.value_handle = uuid;
    }

    void handle_evt(const uint16_t conn_handle,
                    ble_gatts_evt_write_t const *const evt) {
      if (handler) {
        handler(conn_handle, evt, *this);
      } else {
        NRF_LOG_ERROR("Handler for %04hhx invalid.", uuid);
      }
    }

    uint8_t *ptr_value() {
      return (ptr == nullptr) ? reinterpret_cast<uint8_t *>(&value) : ptr;
    }
  };

  static const constexpr uint16_t CONFIG_FILE_ID = 0x0001;
  static const constexpr uint16_t CONFIG_RECORD_ID = 0x0001;

  config_t settings = defaults;
  std::array<struct descriptor, SF_UUID_LAST - SF_UUID_FIRST + 3> ptr_table;

  static_assert(size_in_words(config_t{}) <= FDS_VIRTUAL_PAGE_SIZE - 14,
                "Settings data does not fit in a single FDS page");

  void print_settings() const {
    NRF_LOG_RAW_INFO("Settings:\n");
    NRF_LOG_RAW_INFO(
        "Min duty (cold): %hhu, Min duty (warm): %hhu, Max duty: %hhu\n",
        settings.min_duty_cold, settings.min_duty_warm, settings.max_duty);
    NRF_LOG_RAW_INFO("Preferred ANT device: %hu (%04hx)\n", settings.ant_device,
                     settings.ant_device);
    NRF_LOG_RAW_INFO("Map:  BPM\n");
    NRF_LOG_RAW_INFO("T(°C) 105 110 115 120 125 130 135 140 145\n");
    for (int i = 0; i < 5; ++i) {
      const int t = 10 + i * 5;
      NRF_LOG_RAW_INFO("%3u  ", t);
      for (unsigned off = 0; off < 9; ++off) {
        NRF_LOG_RAW_INFO(" %3u", settings.map[9 * i + off]);
      }
      NRF_LOG_RAW_INFO("\n");
    }
  }

  fds_record_t get_fds_record() const {
    return fds_record_t{
        .file_id = CONFIG_FILE_ID,
        .key = CONFIG_RECORD_ID,
        .data = {.p_data = &settings, .length_words = size_in_words(settings)}};
  }

  ret_code_t find_record(fds_record_desc_t *desc) {
    fds_find_token_t tok = {0};
    // TODO iterate, in case CONFIG_RECORD_ID is not unique.
    return fds_record_find(CONFIG_FILE_ID, CONFIG_RECORD_ID, desc, &tok);
  }

  void write_config() {
    auto rec = get_fds_record();
    const ret_code_t err = fds_record_write(NULL, &rec);
    NRF_LOG_INFO("Writing config: %d", err);
    APP_ERROR_CHECK(err);
    // TODO: handle:
    //  FDS_ERR_NO_SPACE_IN_QUEUES
    //  FDS_ERR_NO_SPACE_IN_FLASH
  }

  void update_config() {
    fds_record_desc_t desc = {0};
    ret_code_t rc = find_record(&desc);
    if (rc == NRF_SUCCESS || rc == FDS_ERR_CRC_CHECK_FAILED) {
      auto rec = get_fds_record();
      rc = fds_record_update(&desc, &rec);
      APP_ERROR_CHECK(rc);
      // TODO: handle:
      //  FDS_ERR_NO_SPACE_IN_QUEUES
      //  FDS_ERR_NO_SPACE_IN_FLASH
    }
    print_settings();
  }

  void load_config() {
    fds_record_desc_t desc = {0};

    ret_code_t rc = find_record(&desc);
    switch (rc) {
    case NRF_SUCCESS: {
      fds_flash_record_t config = {0};
      rc = fds_record_open(&desc, &config);
      if (rc == FDS_ERR_NOT_FOUND) {
        NRF_LOG_INFO("Could not open record: not found. Writing defaults.");
        write_config();
      } else if (rc == NRF_SUCCESS) {
        memcpy(&settings, config.p_data, sizeof(settings));
        NRF_LOG_INFO("Settings loaded from flash.");
        print_settings();
        fds_record_close(&desc);
      } else if (rc == FDS_ERR_CRC_CHECK_FAILED) {
        NRF_LOG_INFO(
            "CRC error loading config from flash. Restoring defaults.");
        fds_record_close(&desc);
        update_config();
      }
    } break;
    case FDS_ERR_NOT_FOUND:
      NRF_LOG_INFO("Config not found. Writing defaults.");
      write_config();
      break;
    default:
      break;
    }
  }

  void update(const uint16_t conn_handle, const uint16_t which_uuid = 0) {
    for (auto &&record : ptr_table) {
      if (record.handle.value_handle == which_uuid || which_uuid == 0) {
        ble_gatts_value_t param = {
            .len = record.size, .offset = 0, .p_value = record.ptr_value()};

        APP_ERROR_CHECK(sd_ble_gatts_value_set(
            conn_handle, record.handle.value_handle, &param));
      }
    }
  }

  uint8_t debug_hr_ = 0;
  uint8_t debug_duty_ = 0;

public:
  PersistentConfiguration()
      : ptr_table({
            descriptor{&settings.ant_device, SF_UUID_ANT_DEVICE,
                       "ANT device ID", GATT_UNIT_NONE,
                       BLE_GATT_CPF_FORMAT_UINT16},
            descriptor{&settings.min_duty_cold, SF_UUID_MIN_DUTY_COLD,
                       "Min cold duty cycle", GATT_UNIT_PERCENT,
                       BLE_GATT_CPF_FORMAT_UINT8},
            descriptor{&settings.min_duty_warm, SF_UUID_MIN_DUTY_WARM,
                       "Min warm duty cycle", GATT_UNIT_PERCENT,
                       BLE_GATT_CPF_FORMAT_UINT8},
            descriptor{&settings.max_duty, SF_UUID_MAX_DUTY, "Max duty cycle",
                       GATT_UNIT_PERCENT, BLE_GATT_CPF_FORMAT_UINT8},
            descriptor{&settings.map, SF_UUID_MAP, "HR map", GATT_UNIT_NONE,
                       BLE_GATT_CPF_FORMAT_STRUCT},
            descriptor{&debug_hr_, SF_UUID_DBG_HR, "Debug HR value",
                       GATT_UNIT_BPM, BLE_GATT_CPF_FORMAT_UINT8,
                       sizeof(debug_hr_), false},
            descriptor{&debug_duty_, SF_UUID_DBG_DUTY, "Debug Duty cycle",
                       GATT_UNIT_PERCENT, BLE_GATT_CPF_FORMAT_UINT8,
                       sizeof(debug_duty_), false},
            descriptor{[this](const uint16_t,
                              ble_gatts_evt_write_t const *const evt,
                              struct descriptor &me) {
                         if (evt->offset != 0 || evt->len != 3) {
                           NRF_LOG_INFO("Wrong offset (%d) or len (%d)",
                                        evt->offset, evt->len);
                           return;
                         }
                         const uint8_t temp_offset = evt->data[0];
                         const uint8_t bpm_offset = evt->data[1];
                         const uint8_t value = evt->data[2];
                         const unsigned idx = temp_offset * hrPoints + bpm_offset;
                         NRF_LOG_INFO("Setting HR map [%u, %u] (%u) to %u",
                                      temp_offset, bpm_offset, idx, value);
                         settings.map[idx] = value;
                         update_config();
                       },
                       SF_UUID_MAP_SINGLE, "HR map single", GATT_UNIT_NONE,
                       BLE_GATT_CPF_FORMAT_UINT24, 3},
            descriptor{
                [this](const uint16_t conn_handle,
                       ble_gatts_evt_write_t const *const,
                       struct descriptor &desc) {
                  settings = defaults;
                  update_config();
                  update(conn_handle);
                },
                SF_UUID_CONF_RESET,
                "Reset configuration values to default",
                GATT_UNIT_NONE,
                BLE_GATT_CPF_FORMAT_UINT8,
                1,
            },
        }) {
    for (auto &record : ptr_table) {
      NRF_LOG_INFO("Size: %d", record.size);
    }
  }

  void init() { load_config(); }

  void add_characteristics_to_service(const uint16_t service_handle,
                                      const uint16_t uuid_type) {
    for (auto &record : ptr_table) {
      const uint16_t size = strlen(record.description);

      ble_add_char_user_desc_t user_desc = {
          .max_size = size,
          .size = size,
          .p_char_user_desc =
              reinterpret_cast<uint8_t*>(const_cast<char *>(record.description)),
          .is_var_len = false,
          .char_props =
              {
                  .broadcast = true,
                  .read = true,
                  .write_wo_resp = false,
                  .write = false,
                  .notify = true,
                  .indicate = false,
                  .auth_signed_wr = false,
              },
          .is_defered_read = false,
          .is_defered_write = false,
          .read_access = SEC_OPEN,
          .is_value_user = false,
      };

      ble_gatts_char_pf_t presentation = {
          .format = record.format,
          .exponent = 0,
          .unit = record.unit,
          .name_space = 1,
          .desc = 0,
      };

      ble_add_char_params_t char_params{};

      char_params.uuid = record.uuid;
      char_params.uuid_type = uuid_type;
      char_params.max_len = record.size;
      char_params.init_len = record.size;
      char_params.p_init_value = (record.ptr == nullptr)
                                     ? reinterpret_cast<uint8_t*>(&record.value)
                                     : record.ptr;
      char_params.is_var_len = false;
      char_params.char_props.read = true;
      char_params.char_props.write = true;

      char_params.read_access = SEC_OPEN;
      char_params.write_access = SEC_OPEN;

      char_params.p_user_descr = &user_desc;
      if (record.format != BLE_GATT_CPF_FORMAT_RFU) {
        char_params.p_presentation_format = &presentation;
      }
      ret_code_t err_code =
          characteristic_add(service_handle, &char_params, &record.handle);
      NRF_LOG_INFO("Adding UUID %04hhx, sz %u: %u", record.uuid, record.size,
                   err_code);
      APP_ERROR_CHECK(err_code);
    }
  }

  auto ANT_Device() const { return settings.ant_device; }
  uint8_t Min_Duty_cold() const { return settings.min_duty_cold; }
  uint8_t Min_Duty_warm() const { return settings.min_duty_warm; }
  uint8_t Max_Duty() const { return settings.max_duty; }

  void update(const uint16_t conn_handle,
              ble_gatts_evt_write_t const *const write_evt) {
    // UUIDs are sequential
    if (write_evt->uuid.uuid >= SF_UUID_FIRST &&
        write_evt->uuid.uuid <= SF_UUID_LAST) {
      const auto index = write_evt->uuid.uuid - SF_UUID_FIRST;
      descriptor &desc = ptr_table[index];
      uint8_t *dst = desc.ptr;

      const auto offset = write_evt->offset;
      const auto len = write_evt->len;
      const auto src = write_evt->data;
      if (dst != nullptr) {
        memcpy(dst + offset, src, len);
        NRF_LOG_INFO("Set %s (%04hx) to %d (%x)", desc.description, desc.uuid,
                     static_cast<uint32_t>(*desc.ptr),
                     static_cast<uint32_t>(*desc.ptr));
        if (desc.persistent) {
          update_config();
        }
      } else {
        NRF_LOG_ERROR("Storage location for UUID %04hx not initialized.",
                      write_evt->uuid.uuid);
      }
    } else if (write_evt->uuid.uuid >= SF_UUID_HANDLER_FIRST &&
               write_evt->uuid.uuid <= SF_UUID_HANDLER_LAST) {
      const auto index = (SF_UUID_LAST - SF_UUID_FIRST) + 1 +
                         (write_evt->uuid.uuid - SF_UUID_HANDLER_FIRST);
      NRF_LOG_INFO("Handler idx %u", index);
      descriptor &desc = ptr_table[index];
      desc.handle_evt(conn_handle, write_evt);
    } else {
      NRF_LOG_ERROR("Write to unknown UUID %04hx", write_evt->uuid.uuid);
    }
  }

  void update(const uint16_t conn_handle,
              const enum ConfigurationSetting setting, const unsigned value) {
    if (setting >= 0 && setting < ptr_table.size()) {
      const auto index = setting;
      auto &desc = ptr_table[setting];
      memcpy(desc.ptr, &value, desc.size);
      // TODO: proper output formatting depending on desc.size
      NRF_LOG_INFO("Set %s (%04hx) to %d (%x)", ptr_table[index].description,
                   ptr_table[index].uuid,
                   *reinterpret_cast<uint16_t *>(ptr_table[index].ptr),
                   *reinterpret_cast<uint16_t *>(ptr_table[index].ptr));
      // update flash
      update_config();
      // update BLE
      update(conn_handle, desc.handle.value_handle /* UUID */);
    } else {
      NRF_LOG_ERROR("Config setting %u out of bounds.", setting);
    }
  }
};

const constexpr PersistentConfiguration::config_t PersistentConfiguration::defaults;
static PersistentConfiguration configuration;

// params: detected line frequency

void ble_sfc_init_static(ble_sfc_t *handle) {
  handle->line_freq_sim_value = 50.0f;
  handle->line_freq_sim_value_updated = false;

  handle->line_sim_value = 0;
  handle->line_sim_updated = false;

  handle->temperature = 0;
}

ble_uuid_t ble_sfc_t::get_service_uuid() const {
  if (uuid_type == 0) {
    NRF_LOG_ERROR("UUID Type not initialized. Service not added?");
  }
  return {SF_UUID_SERVICE, uuid_type};
}

ret_code_t ble_sfc_init(ble_sfc_t *handle) {
  configuration.init();

  ret_code_t err_code;
  /* Add service */
  {
    ble_uuid128_t base_uuid = SF_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &handle->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid_t ble_uuid = {.uuid = SF_UUID_SERVICE, .type = handle->uuid_type};

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid,
                                        &handle->service_handle);
    if (err_code != NRF_SUCCESS) {
      return err_code;
    }
  }

  {
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(char_params));

#if 0
    uint8_t name_buf[] = "Line Freq";

    ble_add_char_user_desc_t user_desc = {
        .max_size = sizeof(name_buf),
        .size = sizeof(name_buf),
        .p_char_user_desc = name_buf,
        .is_var_len = false,
        .char_props =
            {
                .broadcast = true,
                .read = true,
                .write_wo_resp = false,
                .write = false,
                .notify = true,
                .indicate = false,
                .auth_signed_wr = false,
            },
        .is_defered_read = false,
        .is_defered_write = false,
        .read_access = SEC_OPEN,
        .is_value_user = false,
    };
#endif
  
    float line_frequency = 120;

    ble_gatts_char_pf_t presentation = {
        .format = BLE_GATT_CPF_FORMAT_FLOAT32,
        .exponent = 0,
        .unit = 0x2722, // Hz
        .name_space = 1,
        .desc = 0,
    };

    char_params.uuid = SF_UUID_LINE;
    char_params.uuid_type = handle->uuid_type;
    char_params.max_len = sizeof(float);
    char_params.init_len = sizeof(float);
    char_params.p_init_value = (uint8_t*)&line_frequency;
    char_params.is_var_len = false;
    char_params.char_props.read = true;
    char_params.char_props.notify = true;
//    char_params.p_cccd_md = &cccd_md;

    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
    char_params.cccd_write_access = SEC_OPEN;

    //char_params.p_user_descr = &user_desc;
    char_params.p_presentation_format = &presentation;

    err_code = characteristic_add(handle->service_handle, &char_params,
                                  &(handle->line_frequency));
    if (err_code != NRF_SUCCESS) {
      return err_code;
    }
  }

  {
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(char_params));

#if 0
    uint8_t name_buf[] = "Line Freq";

    ble_add_char_user_desc_t user_desc = {
        .max_size = sizeof(name_buf),
        .size = sizeof(name_buf),
        .p_char_user_desc = name_buf,
        .is_var_len = false,
        .char_props =
            {
                .broadcast = true,
                .read = true,
                .write_wo_resp = false,
                .write = false,
                .notify = true,
                .indicate = false,
                .auth_signed_wr = false,
            },
        .is_defered_read = false,
        .is_defered_write = false,
        .read_access = SEC_OPEN,
        .is_value_user = false,
    };
#endif
  

    ble_gatts_char_pf_t presentation = {
        .format = BLE_GATT_CPF_FORMAT_UINT8,
        .exponent = 0,
        .unit = 0x272F, // Temperature in deg C
        .name_space = 1,
        .desc = 0,
    };

    char_params.uuid = SF_UUID_TEMP;
    char_params.uuid_type = handle->uuid_type;
    char_params.max_len = sizeof(uint8_t);
    char_params.init_len = sizeof(uint8_t);
    char_params.p_init_value = &handle->temperature;
    char_params.is_var_len = false;
    char_params.char_props.read = true;
    char_params.char_props.notify = true;
//    char_params.p_cccd_md = &cccd_md;

    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
    char_params.cccd_write_access = SEC_OPEN;

    //char_params.p_user_descr = &user_desc;
    char_params.p_presentation_format = &presentation;

    err_code = characteristic_add(handle->service_handle, &char_params,
                                  &(handle->temperature_char));
    if (err_code != NRF_SUCCESS) {
      return err_code;
    }
  }

#if 0
  if(0) {
    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(char_params));

    ble_gatts_char_pf_t presentation = {
        .format = BLE_GATT_CPF_FORMAT_FLOAT32,
        .exponent = 0,
        .unit = 0x2722, // Hz
        .name_space = 1,
        .desc = 0,
    };

    char_params.uuid = SF_UUID_LINE_SIM;
    char_params.uuid_type = handle->uuid_type;
    char_params.max_len = sizeof(float);
    char_params.init_len = sizeof(float);
    char_params.p_init_value = (uint8_t*)&handle->line_freq_sim_value;
    char_params.is_var_len = false;
    char_params.char_props.read = true;
    char_params.char_props.write = true;

    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
#if 0
    char_params.cccd_write_access = SEC_OPEN;
#endif
    //char_params.p_user_descr = &user_desc;
    char_params.p_presentation_format = &presentation;

    err_code = characteristic_add(handle->service_handle, &char_params,
                                  &(handle->line_frequency_sim));
    if (err_code != NRF_SUCCESS) {
      return err_code;
    }
  }

  if (0) {
    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(char_params));

    char_params.uuid = SF_UUID_SIM;
    char_params.uuid_type = handle->uuid_type;
    char_params.max_len = sizeof(handle->line_sim_value);
    char_params.init_len = sizeof(handle->line_sim_value);
    char_params.p_init_value = &handle->line_sim_value;
    char_params.is_var_len = false;
    char_params.char_props.read = true;
    char_params.char_props.write = true;

    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
#if 0
    char_params.cccd_write_access = SEC_OPEN;
#endif
    //char_params.p_user_descr = &user_desc;

    err_code = characteristic_add(handle->service_handle, &char_params,
                                  &(handle->line_sim));
    if (err_code != NRF_SUCCESS) {
      return err_code;
    }
  }
#endif

  configuration.add_characteristics_to_service(handle->service_handle,
                                               handle->uuid_type);
  return NRF_SUCCESS;
}

void ble_sfc_t::setting(const enum ConfigurationSetting setting,
                        const unsigned value) const {
  configuration.update(conn_handle_, setting, value);
}

unsigned ble_sfc_t::setting(const enum ConfigurationSetting setting) const {
  switch (setting) {
  case ANT_DEVICE:
    return configuration.ANT_Device();
  case MIN_DUTY_COLD:
    return configuration.Min_Duty_cold();
  case MIN_DUTY_WARM:
    return configuration.Min_Duty_warm();
  case MAX_DUTY:
    return configuration.Max_Duty();
  case DEBUG_HR:
    return configuration.debug_hr_;
  case DEBUG_DUTY:
    return configuration.debug_duty_;
  }

  return 0;
}

unsigned ble_sfc_t::map(const unsigned temp, const unsigned bpm) const {
  return configuration.map(temp, bpm);
}

void ble_sfc_t::send_ble_notification(uint8_t const *const ptr, const uint16_t len,
                           const uint16_t value_handle) {

  if (ble_disconnected()) {
    return;
  }

  ble_gatts_hvx_params_t hvx_params{0};

  uint16_t length = len;
  hvx_params.handle = value_handle;
  hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.offset = 0;
  hvx_params.p_len = &length;
  hvx_params.p_data = ptr;

  // returns an error if notifications are not enabled.
  // could check with sd_ble_gatts_value_get() or track CCCD state (stored for
  // bonded devices)
  sd_ble_gatts_hvx(conn_handle_, &hvx_params);
  // update the value, too
  //update_gatts_value(value, value_handle);
}

void ble_sfc_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
  ble_sfc_t *sfc = (ble_sfc_t *)p_context;

  NRF_LOG_RAW_INFO("BLE: %x\n", p_ble_evt->header.evt_id);

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    sfc->ble_connect(p_ble_evt->evt.gap_evt.conn_handle);
    break;
  case BLE_GAP_EVT_DISCONNECTED:
    sfc->ble_disconnect();
    break;
  case BLE_GATTS_EVT_WRITE: {
    ble_gatts_evt_write_t const *p_evt_write =
        &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == sfc->line_frequency_sim.value_handle) {
      // Assert offset 0
      // assert len 4
      // assert uuid matches
      /* min(sizeof(float), len) */
      memcpy(&sfc->line_freq_sim_value, p_evt_write->data, sizeof(float));
      NRF_LOG_INFO("Update simulated line freq to " NRF_LOG_FLOAT_MARKER,
                   NRF_LOG_FLOAT(sfc->line_freq_sim_value));
      sfc->line_freq_sim_value_updated = true;
      break;
    }

    if (p_evt_write->handle == sfc->line_sim.value_handle) {
      // Assert offset 0
      // assert len 4
      // assert uuid matches
      /* min(sizeof(float), len) */
      memcpy(&sfc->line_sim_value, p_evt_write->data, sizeof(uint8_t));
      sfc->line_sim_updated = true;
      break;
    }

    configuration.update(sfc->conn_handle_, p_evt_write);
  } break;
  default:
    break;
  }
}

