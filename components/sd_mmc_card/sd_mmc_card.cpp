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

int constexpr SD_OCR_SDHC_CAP = (1 << 30);  // value defined in esp-idf/components/sdmmc/include/sd_protocol_defs.h
#endif

namespace esphome {
namespace sd_mmc_card {

static const char *TAG = "sd_mmc_card";

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
void SdMmc::setup() {
  // Enable SDCard power if power control pin is configured
  if (this->power_ctrl_pin_ != nullptr) {
    this->power_ctrl_pin_->setup();
    
  }
    // Declare max_files here with a default value
  size_t max_files = 16; // or any other appropriate value
  
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false, 
      .max_files = max_files, 
      .allocation_unit_size = 16 * 1024
  };

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = 40000;  // Limite la fréquence à 40 MHz
  //host.flags &= ~SDMMC_HOST_FLAG_DDR;  // ⛔ Désactiver DDR (par défaut il peut être actif si le matériel le permet)
  host.flags |= SDMMC_HOST_FLAG_DDR;  // ✅ Activer DDR si tu veux tester la vitesse maximale (avec bus_width = 4)
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

  // Enable internal pullups on enabled pins
  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  // Try to mount the SD card
  esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);
  
  // Implementation like Espressif's periph_sdcard_init
  int retry_time = 5;
  bool mount_flag = false;
  
  // First attempt already done above, now we'll retry if needed
  if (ret == ESP_OK) {
    mount_flag = true;
  } else {
    // Retry logic without using delays
    while (--retry_time > 0) {
      ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);
      if (ret == ESP_OK) {
        mount_flag = true;
        break;
      }
    }
  }

  if (!mount_flag) {
    if (ret == ESP_FAIL) {
      this->init_error_ = ErrorCode::ERR_MOUNT;
      ESP_LOGE(TAG, "Failed to mount filesystem on SD card");
    } else {
      this->init_error_ = ErrorCode::ERR_NO_CARD;
      ESP_LOGE(TAG, "No SD card detected");
    }
    mark_failed();
    return;
  }

#ifdef USE_TEXT_SENSOR
  if (this->sd_card_type_text_sensor_ != nullptr)
    this->sd_card_type_text_sensor_->publish_state(sd_card_type());
#endif

  update_sensors();
}
#endif

void SdMmc::write_file(const char *path, const uint8_t *buffer, size_t len) {
  ESP_LOGV(TAG, "Writing to file: %s", path);
  this->write_file(path, buffer, len, "w");
}

void SdMmc::append_file(const char *path, const uint8_t *buffer, size_t len) {
  ESP_LOGV(TAG, "Appending to file: %s", path);
  this->write_file(path, buffer, len, "a");
}

#ifdef USE_ESP_IDF
void SdMmc::write_file(const char *path, const uint8_t *buffer, size_t len, const char *mode) {
  std::string absolut_path = build_path(path);
  FILE *file = NULL;
  file = fopen(absolut_path.c_str(), mode);
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
  }
  bool ok = fwrite(buffer, 1, len, file);
  if (!ok) {
    ESP_LOGE(TAG, "Failed to write to file");
  }
  fclose(file);
  this->update_sensors();
}

void SdMmc::write_file_chunked(const char *path, const uint8_t *buffer, size_t len, size_t chunk_size) {
  std::string absolut_path = build_path(path);
  FILE *file = NULL;
  file = fopen(absolut_path.c_str(), "a");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open file for chunked writing");
    return;
  }

  size_t written = 0;
  while (written < len) {
    size_t to_write = std::min(chunk_size, len - written);
    bool ok = fwrite(buffer + written, 1, to_write, file);
    if (!ok) {
      ESP_LOGE(TAG, "Failed to write chunk to file");
      break;
    }
    written += to_write;
  }
  fclose(file);
  this->update_sensors();
}
#else
void SdMmc::write_file_chunked(const char *path, const uint8_t *buffer, size_t len, size_t chunk_size) {
  ESP_LOGV(TAG, "Writing chunked to file: %s", path);
  size_t written = 0;
  while (written < len) {
    size_t to_write = std::min(chunk_size, len - written);
    this->write_file(path, buffer + written, to_write, "a");
    written += to_write;
  }
}
#endif

std::vector<std::string> SdMmc::list_directory(const char *path, uint8_t depth) {
  std::vector<std::string> list;
  std::vector<FileInfo> infos = list_directory_file_info(path, depth);
  std::transform(infos.cbegin(), infos.cend(), list.begin(), [](FileInfo const &info) { return info.path; });
  return list;
}

std::vector<std::string> SdMmc::list_directory(std::string path, uint8_t depth) {
  return this->list_directory(path.c_str(), depth);
}

std::vector<FileInfo> SdMmc::list_directory_file_info(const char *path, uint8_t depth) {
  std::vector<FileInfo> list;
  list_directory_file_info_rec(path, depth, list);
  return list;
}

std::vector<FileInfo> SdMmc::list_directory_file_info(std::string path, uint8_t depth) {
  return this->list_directory_file_info(path.c_str(), depth);
}

#ifdef USE_ESP_IDF
std::vector<FileInfo> &SdMmc::list_directory_file_info_rec(const char *path, uint8_t depth,
                                                           std::vector<FileInfo> &list) {
  ESP_LOGV(TAG, "Listing directory file info: %s\n", path);
  std::string absolut_path = build_path(path);
  DIR *dir = opendir(absolut_path.c_str());
  if (!dir) {
    ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
    return list;
  }
  char entry_absolut_path[FILE_PATH_MAX];
  char entry_path[FILE_PATH_MAX];
  const size_t dirpath_len = MOUNT_POINT.size();
  size_t entry_path_len = strlen(path);
  strlcpy(entry_path, path, sizeof(entry_path));
  strlcpy(entry_path + entry_path_len, "/", sizeof(entry_path) - entry_path_len);
  entry_path_len = strlen(entry_path);

  strlcpy(entry_absolut_path, MOUNT_POINT.c_str(), sizeof(entry_absolut_path));
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    size_t file_size = 0;
    strlcpy(entry_path + entry_path_len, entry->d_name, sizeof(entry_path) - entry_path_len);
    strlcpy(entry_absolut_path + dirpath_len, entry_path, sizeof(entry_absolut_path) - dirpath_len);
    if (entry->d_type != DT_DIR) {
      struct stat info;
      if (stat(entry_absolut_path, &info) < 0) {
        ESP_LOGE(TAG, "Failed to stat file: %s '%s' %s", strerror(errno), entry->d_name, entry_absolut_path);
      } else {
        file_size = info.st_size;
      }
    }
    list.emplace_back(entry_path, file_size, entry->d_type == DT_DIR);
    if (entry->d_type == DT_DIR && depth)
      list_directory_file_info_rec(entry_absolut_path, depth - 1, list);
  }
  closedir(dir);
  return list;
}

bool SdMmc::is_directory(const char *path) {
  std::string absolut_path = build_path(path);
  DIR *dir = opendir(absolut_path.c_str());
  if (dir) {
    closedir(dir);
  }
  return dir != nullptr;
}

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

bool SdMmc::create_directory(const char *path) {
  ESP_LOGV(TAG, "Create directory: %s", path);
  std::string absolut_path = build_path(path);
  if (mkdir(absolut_path.c_str(), 0777) < 0) {
    ESP_LOGE(TAG, "Failed to create a new directory: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

bool SdMmc::remove_directory(const char *path) {
  ESP_LOGV(TAG, "Remove directory: %s", path);
  if (!this->is_directory(path)) {
    ESP_LOGE(TAG, "Not a directory");
    return false;
  }
  std::string absolut_path = build_path(path);
  if (remove(absolut_path.c_str()) != 0) {
    ESP_LOGE(TAG, "Failed to remove directory: %s", strerror(errno));
  }
  this->update_sensors();
  return true;
}

bool SdMmc::delete_file(const char *path) {
  ESP_LOGV(TAG, "Delete File: %s", path);
  if (this->is_directory(path)) {
    ESP_LOGE(TAG, "Not a file");
    return false;
  }
  std::string absolut_path = build_path(path);
  if (remove(absolut_path.c_str()) != 0) {
    ESP_LOGE(TAG, "Failed to remove file: %s", strerror(errno));
  }
  this->update_sensors();
  return true;
}

std::vector<uint8_t> SdMmc::read_file(char const *path) {
  ESP_LOGV(TAG, "Read File: %s", path);

  std::string absolut_path = build_path(path);
  FILE *file = nullptr;
  file = fopen(absolut_path.c_str(), "rb");
  if (file == nullptr) {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> res;
  size_t fileSize = this->file_size(path);
  res.resize(fileSize);
  size_t len = fread(res.data(), 1, fileSize, file);
  fclose(file);
  if (len < 0) {
    ESP_LOGE(TAG, "Failed to read file: %s", strerror(errno));
    return std::vector<uint8_t>();
  }

  return res;
}

// Lecture d'un fichier en une seule fois (existant)
std::vector<uint8_t> SdMmc::read_file_chunked(const char *path, size_t offset, size_t chunk_size) {
    std::string absolut_path = build_path(path);
    FILE *file = fopen(absolut_path.c_str(), "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", absolut_path.c_str());
        return {};
    }

    std::unique_ptr<FILE, decltype(&fclose)> file_guard(file, fclose);

    if (fseek(file, offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Failed to seek to position %zu in file: %s (errno: %d)", offset, absolut_path.c_str(), errno);
        return {};
    }

    std::vector<uint8_t> res(chunk_size);
    size_t read = fread(res.data(), 1, chunk_size, file);
    res.resize(read); // Ajuste la taille

    return res;
}

// Nouvelle fonction pour le streaming
void SdMmc::read_file_stream(const char *path, size_t offset, size_t chunk_size, 
                             std::function<void(const uint8_t*, size_t)> callback) {
    std::string absolut_path = build_path(path);
    FILE *file = fopen(absolut_path.c_str(), "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", absolut_path.c_str());
        return;
    }

    std::unique_ptr<FILE, decltype(&fclose)> file_guard(file, fclose);

    if (fseek(file, offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Failed to seek to position %zu in file: %s (errno: %d)", offset, absolut_path.c_str(), errno);
        return;
    }

    std::vector<uint8_t> buffer(chunk_size);
    size_t read;
    
    while ((read = fread(buffer.data(), 1, chunk_size, file)) > 0) {
        callback(buffer.data(), read);  // Envoie les données par callback
    }

    if (ferror(file)) {
        ESP_LOGE(TAG, "Error reading file: %s", absolut_path.c_str());
    }
}

#endif
size_t SdMmc::file_size(std::string const &path) { return this->file_size(path.c_str()); }

bool SdMmc::is_directory(std::string const &path) { return this->is_directory(path.c_str()); }

bool SdMmc::delete_file(std::string const &path) { return this->delete_file(path.c_str()); }

std::vector<uint8_t> SdMmc::read_file(std::string const &path) { return this->read_file(path.c_str()); }

std::vector<uint8_t> SdMmc::read_file_chunked(std::string const &path, size_t offset, size_t chunk_size) {
  return this->read_file_chunked(path.c_str(), offset, chunk_size);
}

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

FileInfo::FileInfo(std::string const &path, size_t size, bool is_directory)
    : path(path), size(size), is_directory(is_directory) {}

}  // namespace sd_mmc_card
}  // namespace esphome


