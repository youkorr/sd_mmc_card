#include "ftp_http_proxy.h"
#include "esp_log.h"
#include <lwip/sockets.h>
#include <netdb.h>
#include <cstring>
#include <arpa/inet.h>
#include "esp_task_wdt.h"

static const char *TAG = "ftp_proxy";

// Taille du buffer optimisée pour les transferts
#define DOWNLOAD_BUFFER_SIZE 8192
// Nombre maximal d'octets à transférer avant de réinitialiser le WDT
#define WDT_RESET_THRESHOLD (1024 * 512) // 512 KB

namespace esphome {
namespace ftp_http_proxy {

void FTPHTTPProxy::setup() {
  ESP_LOGI(TAG, "Initialisation du proxy FTP/HTTP");
  
  // Configuration du WDT avec un délai plus long pour les gros fichiers
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 60000,  // 60 secondes au lieu de 30
    .idle_core_mask = 0,  // Pas de cores idle 
    .trigger_panic = false // Ne pas déclencher de panique
  };
  esp_task_wdt_init(&wdt_config);
  
  // Obtention du handle de la tâche courante
  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  
  // Enregistrement de la tâche avec le handle spécifique
  esp_task_wdt_add(current_task);
  
  ESP_LOGI(TAG, "Task Watchdog configuré pour la tâche principale");
  
  this->setup_http_server();
}

void FTPHTTPProxy::loop() {
  // Réinitialiser périodiquement le WDT dans la boucle principale
  esp_task_wdt_reset();
}

bool FTPHTTPProxy::connect_to_ftp() {
  struct hostent *ftp_host = gethostbyname(ftp_server_.c_str());
  if (!ftp_host) {
    ESP_LOGE(TAG, "Échec de la résolution DNS");
    return false;
  }

  sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) {
    ESP_LOGE(TAG, "Échec de création du socket : %d", errno);
    return false;
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(21);
  server_addr.sin_addr.s_addr = *((unsigned long *)ftp_host->h_addr);

  if (::connect(sock_, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
    ESP_LOGE(TAG, "Échec de connexion FTP : %d", errno);
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  char buffer[256];
  int bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "220 ")) {
    ESP_LOGE(TAG, "Message de bienvenue FTP non reçu");
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  // Réinitialisation du watchdog avant l'authentification
  esp_task_wdt_reset();

  // Authentification
  snprintf(buffer, sizeof(buffer), "USER %s\r\n", username_.c_str());
  send(sock_, buffer, strlen(buffer), 0);
  recv(sock_, buffer, sizeof(buffer) - 1, 0);
  esp_task_wdt_reset();

  snprintf(buffer, sizeof(buffer), "PASS %s\r\n", password_.c_str());
  send(sock_, buffer, strlen(buffer), 0);
  recv(sock_, buffer, sizeof(buffer) - 1, 0);
  esp_task_wdt_reset();

  // Mode binaire
  send(sock_, "TYPE I\r\n", 8, 0);
  recv(sock_, buffer, sizeof(buffer) - 1, 0);
  esp_task_wdt_reset();

  return true;
}

bool FTPHTTPProxy::download_file(const std::string &remote_path, httpd_req_t *req) {
  int data_sock = -1;
  bool success = false;
  char *pasv_start = nullptr;
  int data_port = 0;
  int ip[4], port[2]; 
  char buffer[DOWNLOAD_BUFFER_SIZE];  // Buffer plus grand pour améliorer les performances
  int bytes_received;
  size_t total_transferred = 0;
  const char *ext = nullptr;

  if (!connect_to_ftp()) {
    ESP_LOGE(TAG, "Échec de connexion au serveur FTP");
    goto cleanup;
  }

  // Configuration des timeouts pour les sockets
  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  send(sock_, "PASV\r\n", 6, 0);
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "227 ")) {
    ESP_LOGE(TAG, "Échec de passage en mode passif");
    goto cleanup;
  }
  esp_task_wdt_reset();

  pasv_start = strchr(buffer, '(');
  if (!pasv_start) {
    ESP_LOGE(TAG, "Format de réponse PASV invalide");
    goto cleanup;
  }

  sscanf(pasv_start, "(%d,%d,%d,%d,%d,%d)", &ip[0], &ip[1], &ip[2], &ip[3], &port[0], &port[1]);
  data_port = port[0] * 256 + port[1];

  data_sock = ::socket(AF_INET, SOCK_STREAM, 0);
  if (data_sock < 0) {
    ESP_LOGE(TAG, "Échec de création du socket de données");
    goto cleanup;
  }

  // Configuration du timeout pour le socket de données
  setsockopt(data_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  struct sockaddr_in data_addr;
  memset(&data_addr, 0, sizeof(data_addr));
  data_addr.sin_family = AF_INET;
  data_addr.sin_port = htons(data_port);
  data_addr.sin_addr.s_addr = htonl((ip[0] << 24) | (ip[1] << 16) | (ip[2] << 8) | ip[3]);

  if (::connect(data_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) != 0) {
    ESP_LOGE(TAG, "Échec de connexion au port de données");
    goto cleanup;
  }

  // Définir l'en-tête Content-Type pour MP3 ou autres fichiers si nécessaire
  ext = strrchr(remote_path.c_str(), '.');
  if (ext) {
    if (strcmp(ext, ".mp3") == 0) {
      httpd_resp_set_type(req, "audio/mpeg");
    } else if (strcmp(ext, ".pdf") == 0) {
      httpd_resp_set_type(req, "application/pdf");
    } else if (strcmp(ext, ".jpg") == 0 || strcmp(ext, ".jpeg") == 0) {
      httpd_resp_set_type(req, "image/jpeg");
    } else if (strcmp(ext, ".png") == 0) {
      httpd_resp_set_type(req, "image/png");
    }
    // Ajouter d'autres types selon les besoins
  }

  snprintf(buffer, sizeof(buffer), "RETR %s\r\n", remote_path.c_str());
  send(sock_, buffer, strlen(buffer), 0);

  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "150 ")) {
    ESP_LOGE(TAG, "Échec de la commande RETR");
    goto cleanup;
  }
  
  ESP_LOGI(TAG, "Début du téléchargement de %s", remote_path.c_str());
  
  // Réinitialiser le watchdog avant de commencer le transfert de données
  esp_task_wdt_reset();

  while (true) {
    bytes_received = recv(data_sock, buffer, sizeof(buffer), 0);
    if (bytes_received <= 0) break;

    total_transferred += bytes_received;
    
    // Réinitialisation périodique du WDT pendant le téléchargement
    if (total_transferred >= WDT_RESET_THRESHOLD) {
      ESP_LOGD(TAG, "Transfert en cours: %zu octets", total_transferred);
      esp_task_wdt_reset();
      total_transferred = 0; // Réinitialiser le compteur
    }

    if (httpd_resp_send_chunk(req, buffer, bytes_received) != ESP_OK) {
      ESP_LOGE(TAG, "Échec d'envoi au client: %d", errno);
      break;
    }
  }

  // Réinitialiser le watchdog après le transfert complet
  esp_task_wdt_reset();

  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received > 0 && strstr(buffer, "226 ")) {
    success = true;
    ESP_LOGI(TAG, "Téléchargement terminé avec succès");
  }

  httpd_resp_send_chunk(req, NULL, 0);

cleanup:
  if (!success) {
    ESP_LOGE(TAG, "Erreur lors du téléchargement");
  }
  
  if (data_sock != -1) 
    ::close(data_sock);
  
  if (sock_ != -1) {
    send(sock_, "QUIT\r\n", 6, 0);
    ::close(sock_);
    sock_ = -1;
  }
  
  return success;
}

esp_err_t FTPHTTPProxy::http_req_handler(httpd_req_t *req) {
  auto *proxy = (FTPHTTPProxy *)req->user_ctx;
  std::string requested_path = req->uri;

  if (!requested_path.empty() && requested_path[0] == '/') {
    requested_path.erase(0, 1);
  }

  // Réinitialisation du watchdog au début de chaque requête HTTP
  esp_task_wdt_reset();

  for (const auto &configured_path : proxy->remote_paths_) {
    if (requested_path == configured_path) {
      ESP_LOGI(TAG, "Téléchargement demandé: %s", requested_path.c_str());
      if (proxy->download_file(configured_path, req)) {
        return ESP_OK;
      } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Échec du téléchargement");
        return ESP_FAIL;
      }
    }
  }

  httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Fichier non trouvé");
  return ESP_FAIL;
}

void FTPHTTPProxy::setup_http_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = local_port_;
  config.uri_match_fn = httpd_uri_match_wildcard;
  
  // Augmenter la taille maximale des données de la requête HTTP
  config.max_resp_headers = 16;
  config.max_uri_handlers = 8;
  config.stack_size = 8192;  // Augmenter la taille de la pile
  
  if (httpd_start(&server_, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Échec du démarrage du serveur HTTP");
    return;
  }

  httpd_uri_t uri_proxy = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = http_req_handler,
    .user_ctx  = this
  };

  httpd_register_uri_handler(server_, &uri_proxy);
  ESP_LOGI(TAG, "Serveur HTTP démarré sur le port %d", local_port_);
}

}  // namespace ftp_http_proxy
}  // namespace esphome



