#pragma once

#include "esphome.h"
#include <vector>
#include <string>
#include <esp_http_server.h>
#include <lwip/sockets.h>

namespace esphome {
namespace ftp_http_proxy {

class FTPHTTPProxy : public Component {
 public:
  void set_ftp_server(const std::string &server) { ftp_server_ = server; }
  void set_username(const std::string &username) { username_ = username; }
  void set_password(const std::string &password) { password_ = password; }
  void add_remote_path(const std::string &path) { remote_paths_.push_back(path); }
  void set_local_port(uint16_t port) { local_port_ = port; }

  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

 protected:
  std::string ftp_server_;
  std::string username_;
  std::string password_;
  std::vector<std::string> remote_paths_;
  uint16_t local_port_{8000};
  httpd_handle_t server_{nullptr};
  int sock_{-1};
  int ftp_port_ = 21;
  bool send_ftp_command(const std::string &cmd, std::string &response);

  bool connect_to_ftp();
  bool download_file(const std::string &remote_path, httpd_req_t *req);
  void setup_http_server();
  static esp_err_t http_req_handler(httpd_req_t *req);
};

}  // namespace ftp_http_proxy
}  // namespace esphome
