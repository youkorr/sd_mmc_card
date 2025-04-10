#include "sd_mmc_card.h"

#include <algorithm>
#include <vector>
#include <cstdio>

#include "math.h"
#include "esphome/core/log.h"

#ifdef USE_ESP_IDF
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int constexpr SD_OCR_SDHC_CAP = (1 << 30);  // value defined in esp-idf/components/sdmmc/include/sd_protocol_defs.h
#endif

/**
 * @brief SDCARD Function Definition
 */
#define FUNC_SDCARD_EN              (1)
#define SDCARD_OPEN_FILE_NUM_MAX    (5)
#define SDCARD_INTR_GPIO            (-1)
#define SDCARD_PWR_CTRL             GPIO_NUM_43
#define ESP_SD_PIN_CLK              GPIO_NUM_11
#define ESP_SD_PIN_CMD              GPIO_NUM_14
#define ESP_SD_PIN_D0               GPIO_NUM_9
#define ESP_SD_PIN_D1               GPIO_NUM_13
#define ESP_SD_PIN_D2               GPIO_NUM_42
#define ESP_SD_PIN_D3               GPIO_NUM_12
#define ESP_SD_PIN_D4               (-1)
#define ESP_SD_PIN_D5               (-1)
#define ESP_SD_PIN_D6               (-1)
#define ESP_SD_PIN_D7               (-1)
#define ESP_SD_PIN_CD               (-1)
#define ESP_SD_PIN_WP               (-1)

namespace esphome {
namespace sd_mmc_card {

static const char *TAG = "sd_mmc_card";

// SD card configuration helper functions
int8_t get_sdcard_intr_gpio(void) { return SDCARD_INTR_GPIO; }
int8_t get_sdcard_open_file_num_max(void) { return SDCARD_OPEN_FILE_NUM_MAX; }
int8_t get_sdcard_power_ctrl_gpio(void) { return SDCARD_PWR_CTRL; }

#ifdef USE_ESP_IDF
static constexpr size_t FILE_PATH_MAX = ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN;
static const std::string MOUNT_POINT("/sdcard");

std::string build_path(const char *path) { return MOUNT_POINT + path; }
#endif

#ifdef USE_SENSOR
FileSizeSensor::FileSizeSensor(sensor::Sensor *sensor, std::string const &path) : sensor(sensor), path(path) {}
#endif

void SdMmc::loop() {}

void SdMmc::dump_config() {
  ESP_LOGCONFIG(TAG, "SD MMC Component");
  ESP_LOGCONFIG(TAG, "  Mode 1 bit: %s", TRUEFALSE(this->mode_1bit_));
  ESP_LOGCONFIG(TAG, "  CLK Pin: %d", this->clk_pin_);
  ESP_LOGCONFIG(TAG, "  CMD Pin: %d", this->cmd_pin_);
  ESP_LOGCONFIG(TAG, "  DATA0 Pin: %d", this->data0_pin_);
  if (!this->mode_1bit_) {
    ESP_LOGCONFIG(TAG, "  DATA1 Pin: %d", this->data1_pin_);
    ESP_LOGCONFIG(TAG, "  DATA2 Pin: %d", this->data2_pin_);
    ESP_LOGCONFIG(TAG, "  DATA3 Pin: %d", this->data3_pin_);
  }

  if (this->power_ctrl_pin_ != nullptr) {
    LOG_PIN("  Power Ctrl Pin: ", this->power_ctrl_pin_);
  }

#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Used space", this->used_space_sensor_);
  LOG_SENSOR("  ", "Total space", this->total_space_sensor_);
  LOG_SENSOR("  ", "Free space", this->free_space_sensor_);
  for (auto &sensor : this->file_size_sensors_) {
    if (sensor.sensor != nullptr)
      LOG_SENSOR("  ", "File size", sensor.sensor);
  }
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "SD Card Type", this->sd_card_type_text_sensor_);
#endif

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Setup failed : %s", SdMmc::error_code_to_string(this->init_error_).c_str());
    return;
  }
}

#ifdef USE_ESP_IDF
esp_err_t SdMmc::initialize_sdcard() {
  // Enable SDCard power
  if (get_sdcard_power_ctrl_gpio() >= 0) {
    gpio_config_t gpio_cfg = {
      .pin_bit_mask = 1ULL << get_sdcard_power_ctrl_gpio(),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_cfg);
    gpio_set_level(static_cast<gpio_num_t>(get_sdcard_power_ctrl_gpio()), 0);
    
    // Give time for power stabilization
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = get_sdcard_open_file_num_max(),
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  
  // Configure SD card pins using the predefined values
  slot_config.clk = static_cast<gpio_num_t>(ESP_SD_PIN_CLK);
  slot_config.cmd = static_cast<gpio_num_t>(ESP_SD_PIN_CMD);
  slot_config.d0 = static_cast<gpio_num_t>(ESP_SD_PIN_D0);
  slot_config.d1 = static_cast<gpio_num_t>(ESP_SD_PIN_D1);
  slot_config.d2 = static_cast<gpio_num_t>(ESP_SD_PIN_D2);
  slot_config.d3 = static_cast<gpio_num_t>(ESP_SD_PIN_D3);
  
  // Enable internal pullups
  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
  
  esp_err_t ret = ESP_FAIL;
  int retry_time = 5;
  
  while (retry_time--) {
    ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "SD card mounted successfully");
      break;
    } else {
      ESP_LOGW(TAG, "SD card mount attempt failed, retrying... (%d)", retry_time);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }
  
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SD card mount failed after multiple attempts");
    return ESP_FAIL;
  }
  
  return ESP_OK;
}

void SdMmc::setup() {
  // Setup power control pin if specified
  if (this->power_ctrl_pin_ != nullptr) {
    this->power_ctrl_pin_->setup();
  }

  ESP_LOGI(TAG, "Initializing SD card with dedicated pins");
  // Try the new initialization method with predefined pins first
  if (initialize_sdcard() == ESP_OK) {
    ESP_LOGI(TAG, "SD card initialized with dedicated pins");
    #ifdef USE_TEXT_SENSOR
    if (this->sd_card_type_text_sensor_ != nullptr)
      this->sd_card_type_text_sensor_->publish_state(sd_card_type());
    #endif
    
    update_sensors();
    return;
  }
  
  ESP_LOGW(TAG, "Failed to initialize SD card with dedicated pins, falling back to user-configured pins");
  
  // Fall back to the original method if the new one fails
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024};

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  if (this->mode_1bit_) {
    slot_config.width = 1;
  } else {
    slot_config.width = 4;
  }

#ifdef SOC_SDMMC_USE_GPIO_MATRIX
  slot_config.clk = static_cast<gpio_num_t>(this->clk_pin_);
  slot_config.cmd = static_cast<gpio_num_t>(this->cmd_pin_);
  slot_config.d0 = static_cast<gpio_num_t>(this->data0_pin_);

  if (!this->mode_1bit_) {
    slot_config.d1 = static_cast<gpio_num_t>(this->data1_pin_);
    slot_config.d2 = static_cast<gpio_num_t>(this->data2_pin_);
    slot_config.d3 = static_cast<gpio_num_t>(this->data3_pin_);
  }
#endif

  // Enable internal pullups on enabled pins. The internal pullups
  // are insufficient however, please make sure 10k external pullups are
  // connected on the bus. This is for debug / example purpose only.
  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  auto ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      this->init_error_ = ErrorCode::ERR_MOUNT;
    } else {
      this->init_error_ = ErrorCode::ERR_NO_CARD;
    }
    mark_failed();
    return;
  }

  ESP_LOGI(TAG, "SD card initialized with user-configured pins");
#ifdef USE_TEXT_SENSOR
  if (this->sd_card_type_text_sensor_ != nullptr)
    this->sd_card_type_text_sensor_->publish_state(sd_card_type());
#endif

  update_sensors();
}
#endif

size_t SdMmc::file_size(const char *path) {
  std::string absolut_path = build_path(path);
  struct stat info;
  size_t file_size = 0;
  if (stat(absolut_path.c_str(), &info) < 0) {
    ESP_LOGE(TAG, "Failed to stat file: %s", strerror(errno));
    return -1;
  }
  return info.st_size;
}

std::string SdMmc::sd_card_type() const {
  if (this->card_->is_sdio) {
    return "SDIO";
  } else if (this->card_->is_mmc) {
    return "MMC";
  } else {
    return (this->card_->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
  }
  return "UNKNOWN";
}

void SdMmc::update_sensors() {
#ifdef USE_SENSOR
  if (this->card_ == nullptr)
    return;

  FATFS *fs;
  DWORD fre_clust, fre_sect, tot_sect;
  uint64_t total_bytes = -1, free_bytes = -1, used_bytes = -1;
  auto res = f_getfree(MOUNT_POINT.c_str(), &fre_clust, &fs);
  if (!res) {
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    total_bytes = static_cast<uint64_t>(tot_sect) * FF_SS_SDCARD;
    free_bytes = static_cast<uint64_t>(fre_sect) * FF_SS_SDCARD;
    used_bytes = total_bytes - free_bytes;
  }

  if (this->used_space_sensor_ != nullptr)
    this->used_space_sensor_->publish_state(used_bytes);
  if (this->total_space_sensor_ != nullptr)
    this->total_space_sensor_->publish_state(total_bytes);
  if (this->free_space_sensor_ != nullptr)
    this->free_space_sensor_->publish_state(free_bytes);

  for (auto &sensor : this->file_size_sensors_) {
    if (sensor.sensor != nullptr)
      sensor.sensor->publish_state(this->file_size(sensor.path));
  }
#endif
}

size_t SdMmc::file_size(std::string const &path) { return this->file_size(path.c_str()); }

#ifdef USE_SENSOR
void SdMmc::add_file_size_sensor(sensor::Sensor *sensor, std::string const &path) {
  this->file_size_sensors_.emplace_back(sensor, path);
}
#endif

void SdMmc::set_clk_pin(uint8_t pin) { this->clk_pin_ = pin; }

void SdMmc::set_cmd_pin(uint8_t pin) { this->cmd_pin_ = pin; }

void SdMmc::set_data0_pin(uint8_t pin) { this->data0_pin_ = pin; }

void SdMmc::set_data1_pin(uint8_t pin) { this->data1_pin_ = pin; }

void SdMmc::set_data2_pin(uint8_t pin) { this->data2_pin_ = pin; }

void SdMmc::set_data3_pin(uint8_t pin) { this->data3_pin_ = pin; }

void SdMmc::set_mode_1bit(bool b) { this->mode_1bit_ = b; }

void SdMmc::set_power_ctrl_pin(GPIOPin *pin) { this->power_ctrl_pin_ = pin; }

std::string SdMmc::error_code_to_string(SdMmc::ErrorCode code) {
  switch (code) {
    case ErrorCode::ERR_PIN_SETUP:
      return "Failed to set pins";
    case ErrorCode::ERR_MOUNT:
      return "Failed to mount card";
    case ErrorCode::ERR_NO_CARD:
      return "No card found";
    default:
      return "Unknown error";
  }
}

long double convertBytes(uint64_t value, MemoryUnits unit) {
  return value * 1.0 / pow(1024, static_cast<uint64_t>(unit));
}

}  // namespace sd_mmc_card
}  // namespace esphome








