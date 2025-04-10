#include "sd_mmc_card.h"

#include <algorithm>
#include <vector>
#include <cstdio>

#include "math.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h" // Include this for delay function

#ifdef USE_ESP_IDF
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"


// Constantes d'optimisation pour les transferts - ajustées pour stabilité

#define SD_FREQ_HIGH_SPEED SDMMC_FREQ_HIGHSPEED // 40MHz pour SDHC/SDXC
#define SD_FREQ_DEFAULT SDMMC_FREQ_DEFAULT // 20MHz pour SDSC
#define SD_FILE_BUFFER_SIZE 16384 // 16KB
#define SD_DEFAULT_MAX_FILES 5 // Retour à 5 fichiers comme dans la version originale
#define SD_POWER_STABILIZATION_DELAY 250 // 250ms pour stabilisation de l'alimentation

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

void SdMmc::loop() {
  // Rien à faire dans la boucle principale pour l'instant
}

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
  
  ESP_LOGCONFIG(TAG, "  High Speed Mode: %s", this->high_speed_mode_ ? "Enabled" : "Disabled");
  ESP_LOGCONFIG(TAG, "  Frequency: %d MHz", this->high_speed_mode_ ? 40 : 20);

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
  ESP_LOGI(TAG, "Setting up SD MMC component");
  
  // Configuration correcte du pin de contrôle d'alimentation
  if (this->power_ctrl_pin_ != nullptr) {
    ESP_LOGI(TAG, "Setting up power control pin as fixed output");
    this->power_ctrl_pin_->setup();
    this->power_ctrl_pin_->digital_write(true);  // Activer l'alimentation avec une sortie fixe
    
    // Attendre que l'alimentation se stabilise
    ESP_LOGI(TAG, "Waiting for power stabilization (%dms)...", SD_POWER_STABILIZATION_DELAY);
    esphome::delay(SD_POWER_STABILIZATION_DELAY);
    ESP_LOGI(TAG, "Power stabilization complete");
  }

  ESP_LOGI(TAG, "Configuring mount settings");
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false, 
      .max_files = SD_DEFAULT_MAX_FILES, 
      .allocation_unit_size = 16 * 1024};  // 16KB par bloc pour de meilleures performances

  ESP_LOGI(TAG, "Configuring host");
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  
  // Réduire la fréquence initiale pour améliorer la stabilité de l'initialisation
  host.max_freq_khz = SDMMC_FREQ_DEFAULT;
  
  ESP_LOGI(TAG, "Configuring slot");
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  if (this->mode_1bit_) {
    ESP_LOGI(TAG, "Setting 1-bit mode");
    slot_config.width = 1;
    host.flags &= ~SDMMC_HOST_FLAG_4BIT;
  } else {
    ESP_LOGI(TAG, "Setting 4-bit mode");
    slot_config.width = 4;
    host.flags |= SDMMC_HOST_FLAG_4BIT;  // Force le mode 4-bit
  }

#ifdef SOC_SDMMC_USE_GPIO_MATRIX
  ESP_LOGI(TAG, "Setting GPIO pins: CLK=%d, CMD=%d, D0=%d", 
           this->clk_pin_, this->cmd_pin_, this->data0_pin_);
  slot_config.clk = static_cast<gpio_num_t>(this->clk_pin_);
  slot_config.cmd = static_cast<gpio_num_t>(this->cmd_pin_);
  slot_config.d0 = static_cast<gpio_num_t>(this->data0_pin_);

  if (!this->mode_1bit_) {
    ESP_LOGI(TAG, "Setting additional data pins: D1=%d, D2=%d, D3=%d", 
             this->data1_pin_, this->data2_pin_, this->data3_pin_);
    slot_config.d1 = static_cast<gpio_num_t>(this->data1_pin_);
    slot_config.d2 = static_cast<gpio_num_t>(this->data2_pin_);
    slot_config.d3 = static_cast<gpio_num_t>(this->data3_pin_);
  }
#endif

  // Activer les pullups internes sur les broches de données uniquement, pas sur le contrôle d'alimentation
  if (!this->mode_1bit_) {
    // En mode 4-bit, activez les pullups pour la stabilité des données
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    ESP_LOGI(TAG, "Internal pullups enabled for data lines");
  } else {
    // En mode 1-bit, on peut aussi utiliser les pullups pour stabiliser DATA0
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    ESP_LOGI(TAG, "Internal pullups enabled for data line");
  }

  ESP_LOGI(TAG, "Mounting SD card...");
  auto ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      this->init_error_ = ErrorCode::ERR_MOUNT;
      ESP_LOGE(TAG, "Failed to mount filesystem on SD card (%s)", esp_err_to_name(ret));
    } else {
      this->init_error_ = ErrorCode::ERR_NO_CARD;
      ESP_LOGE(TAG, "Failed to initialize the card (%s)", esp_err_to_name(ret));
    }
    mark_failed();
    return;
  }

  ESP_LOGI(TAG, "SD card mounted successfully");
  
  // Augmentation progressive de la vitesse pour les cartes en mode 4-bit
  bool is_high_cap = (this->card_->ocr & SD_OCR_SDHC_CAP) ? true : false;
  
  if (!this->mode_1bit_ && is_high_cap && this->high_speed_mode_) {
    ESP_LOGI(TAG, "Card initialized in 4-bit mode, gradually increasing speed...");
    
    // Première étape à 25 MHz
    ESP_LOGI(TAG, "Stepping up to 25MHz...");
    esp_err_t ret = sdmmc_host_set_card_clk(host.slot, 25000);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Successfully set to 25MHz, waiting for stabilization");
      esphome::delay(50);  // Attendre la stabilisation
      
      // Deuxième étape à vitesse maximale
      ESP_LOGI(TAG, "Stepping up to high speed mode (40MHz)...");
      ret = sdmmc_host_set_card_clk(host.slot, SD_FREQ_HIGH_SPEED);
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Successfully set high speed mode (40MHz)");
      } else {
        ESP_LOGW(TAG, "Failed to set 40MHz, staying at 25MHz: %s", esp_err_to_name(ret));
      }
    } else {
      ESP_LOGW(TAG, "Failed to set 25MHz, staying at default speed: %s", esp_err_to_name(ret));
    }
  } else if (is_high_cap && this->high_speed_mode_) {
    // Mode 1-bit avec carte haute capacité
    ESP_LOGI(TAG, "Setting high speed mode for SDHC/SDXC card (1-bit mode)");
    sdmmc_host_set_card_clk(host.slot, SD_FREQ_HIGH_SPEED);
  }

  // Afficher des informations détaillées sur la carte et la configuration
  ESP_LOGI(TAG, "SD Card Info:");
  ESP_LOGI(TAG, "  Name: %s", this->card_->cid.name);
  ESP_LOGI(TAG, "  Type: %s", this->sd_card_type().c_str());
  ESP_LOGI(TAG, "  Bus Width: %d-bit", this->mode_1bit_ ? 1 : 4);
  ESP_LOGI(TAG, "  Clock Frequency: %d kHz", this->card_->host.max_freq_khz);
  ESP_LOGI(TAG, "  Size: %lluMB", this->card_->csd.capacity / (1024));
  ESP_LOGI(TAG, "  CSD: ver=%d, sector_size=%d, capacity=%lld, read_bl_len=%d",
           this->card_->csd.csd_ver,
           this->card_->csd.sector_size,
           this->card_->csd.capacity,
           this->card_->csd.read_block_len);

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

// Implementation of read_file
bool SdMmc::read_file(const char *path, uint8_t *buffer, size_t &size, size_t offset) {
  std::string full_path = build_path(path);
  FILE *f = fopen(full_path.c_str(), "rb");
  if (f == nullptr) {
    ESP_LOGE(TAG, "Failed to open file %s for reading: %s", path, strerror(errno));
    return false;
  }

  // Positionnement au point de départ demandé
  if (fseek(f, offset, SEEK_SET) != 0) {
    ESP_LOGE(TAG, "Failed to seek in file %s: %s", path, strerror(errno));
    fclose(f);
    return false;
  }

  // Lecture optimisée
  size_t bytes_read = fread(buffer, 1, size, f);
  fclose(f);

  if (bytes_read != size) {
    // C'est normal si on atteint la fin du fichier
    if (feof(f)) {
      size = bytes_read;
      return true;
    }
    ESP_LOGE(TAG, "Failed to read file %s: %s", path, strerror(errno));
    return false;
  }

  return true;
}

// Implementation of read_file_chunked
bool SdMmc::read_file_chunked(const char *path, 
                             std::function<bool(const uint8_t*, size_t)> data_callback,
                             size_t chunk_size, 
                             size_t offset) {
  std::string full_path = build_path(path);
  FILE *f = fopen(full_path.c_str(), "rb");
  if (f == nullptr) {
    ESP_LOGE(TAG, "Failed to open file %s for reading: %s", path, strerror(errno));
    return false;
  }

  // Obtenir la taille du fichier
  fseek(f, 0, SEEK_END);
  size_t file_size = ftell(f);
  
  // Se positionner au bon offset
  fseek(f, offset, SEEK_SET);
  
  // Allouer un buffer pour la lecture par chunks
  uint8_t* buffer = (uint8_t*)malloc(chunk_size);
  if (buffer == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate read buffer");
    fclose(f);
    return false;
  }
  
  size_t bytes_left = file_size - offset;
  bool success = true;
  
  while (bytes_left > 0 && success) {
    size_t to_read = (bytes_left > chunk_size) ? chunk_size : bytes_left;
    size_t bytes_read = fread(buffer, 1, to_read, f);
    
    if (bytes_read > 0) {
      // Appel du callback avec les données lues
      success = data_callback(buffer, bytes_read);
      bytes_left -= bytes_read;
    }
    
    if (bytes_read < to_read) {
      // Erreur de lecture ou fin de fichier prématurée
      if (feof(f)) {
        // Fin de fichier normale
        break;
      } else {
        // Erreur de lecture
        ESP_LOGE(TAG, "Error reading file: %s", strerror(errno));
        success = false;
      }
    }
  }
  
  free(buffer);
  fclose(f);
  return success;
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

void SdMmc::set_high_speed_mode(bool b) { this->high_speed_mode_ = b; }

bool SdMmc::get_high_speed_mode() const { return this->high_speed_mode_; }

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


