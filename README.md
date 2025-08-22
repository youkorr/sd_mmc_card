# WEBDAV ,SD_CARD ,STORAGE IMAGES DIRECT READ FOR SD 

SD MMC cards components for esphome.and webdav for esp32S3
```
## Config

```yaml
esp32:
  board: esp32-s3-devkitc-1
  flash_size: 16MB
  cpu_frequency: 240MHZ
  framework:
    type: esp-idf
    version: recommended
    sdkconfig_options:
      CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240: "y"
      CONFIG_ESP32S3_DATA_CACHE_64KB: "y"
      CONFIG_ESP32S3_DATA_CACHE_LINE_64B: "y"
      CONFIG_FATFS_LFN_STACK: "y"
      CONFIG_LWIP_SO_RCVBUF: "y"
      CONFIG_LWIP_MAX_SOCKETS: "16"
  
```

* **mode_1bit** (Optional, bool): spécifie si le mode 1 bit ou 4 bits est utilisé
* **clk_pin** : (Required, GPIO): broche d'horloge
* **cmd_pin** : (Required, GPIO): broche de commande
* **data0_pin**: (Required, GPIO): broche de données 0
* **data1_pin**: (Optional, GPIO): broche de données 1, utilisée uniquement en mode 4 bits
* **data2_pin**: (Optional, GPIO): broche de données 2, utilisée uniquement en mode 4 bits
* **data3_pin**: (Optional, GPIO): broche de données 3, utilisée uniquement en mode 4 bits
* **crt_pwr**: 



Sample configuration for the  esp32s3:
```yaml
sd_mmc_card:
  id: sd_card
  clk_pin: GPIO11
  cmd_pin: GPIO14
  data0_pin: GPIO9
  data1_pin: GPIO13
  data2_pin: GPIO42
  data3_pin: GPIO12
  mode_1bit: false
  power_ctrl_pin: GPIO43  option [BOX3]

sd_mmc_card:
  id: sd_card
  clk_pin: GPIO11
  cmd_pin: GPIO14
  data0_pin: GPIO9
  mode_1bit: true
  power_ctrl_pin: GPIO43  option [BOX3]
   
``` yaml
webdavbox3:
  id: sd_cards
  root_path: "/sdcard"
  url_prefix: "/"
  port: 81
  username: ""
  password: ""

storage:
    platform: sd_direct
    id: my_storage
    sd_component: sd_card
    root_path: "/sdcard" 
    sd_images:
      - id: testree
        file_path: "/img/sanctuary.jpg"
        resize: 1280x720
        format: rgb565
        byte_order: little_endian 
        auto_load: true

```
