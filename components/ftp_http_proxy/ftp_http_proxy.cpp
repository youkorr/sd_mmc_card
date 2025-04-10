#include "ftp_http_proxy.h"

#include "esphome/core/log.h"
#include "esp_task_wdt.h"
#include <string>
#include <cstdio>

namespace esphome {
namespace ftp_http_proxy {

static const char *const TAG = "ftp_http_proxy";

void FTPHTTPProxy::setup() {
  const esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 30000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true};
  esp_task_wdt_init(&wdt_config);
}

bool FTPHTTPProxy::transfer_file(const std::string &src, const std::string &dst) {
  ESP_LOGI(TAG, "Starting file transfer from %s to %s", src.c_str(), dst.c_str());
  FILE *fin = fopen(src.c_str(), "rb");
  if (!fin) {
    ESP_LOGE(TAG, "Failed to open source file: %s", strerror(errno));
    return false;
  }

  FILE *fout = fopen(dst.c_str(), "wb");
  if (!fout) {
    fclose(fin);
    ESP_LOGE(TAG, "Failed to open destination file: %s", strerror(errno));
    return false;
  }

  static constexpr size_t BUFFER_SIZE = 32 * 1024;
  std::vector<uint8_t> buffer(BUFFER_SIZE);
  size_t total = 0;

  while (!feof(fin)) {
    size_t read = fread(buffer.data(), 1, BUFFER_SIZE, fin);
    if (read == 0)
      break;

    size_t written = fwrite(buffer.data(), 1, read, fout);
    if (written != read) {
      ESP_LOGE(TAG, "Failed to write full buffer (written=%d, read=%d)", (int) written, (int) read);
      fclose(fin);
      fclose(fout);
      return false;
    }

    total += written;
    esp_task_wdt_reset();  // 🛡️ Protection watchdog
  }

  ESP_LOGI(TAG, "Transfer completed. Total bytes: %u", (unsigned int) total);
  fclose(fin);
  fclose(fout);
  return true;
}

}  // namespace ftp_http_proxy
}  // namespace esphome

