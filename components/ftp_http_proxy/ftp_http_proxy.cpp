#include "ftp_http_proxy.h"
#include "esp_log.h"
#include <lwip/sockets.h>
#include <netdb.h>
#include <cstring>
#include <arpa/inet.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include <fcntl.h>

static const char *TAG = "ftp_proxy";

// Structure pour les demandes de transfert FTP
typedef struct {
  std::string remote_path;
  httpd_req_t *req;
  bool completed;
  bool success;
} ftp_transfer_request_t;

// File d'attente pour les demandes de transfert
static QueueHandle_t ftp_queue = NULL;
// Handle de la tâche FTP
static TaskHandle_t ftp_task_handle = NULL;

namespace esphome {
namespace ftp_http_proxy {

// Fonction de tâche FTP
void ftp_task_function(void *pvParameters) {
  FTPHTTPProxy *proxy = static_cast<FTPHTTPProxy*>(pvParameters);
  ftp_transfer_request_t request;
  
  while (true) {
    // Attendre une demande de transfert
    if (xQueueReceive(ftp_queue, &request, portMAX_DELAY) == pdTRUE) {
      ESP_LOGI(TAG, "Tâche FTP: traitement de la demande pour %s", request.remote_path.c_str());
      
      // Effectuer le transfert
      request.success = proxy->process_ftp_transfer(request.remote_path, request.req);
      request.completed = true;
      
      // Petit délai pour s'assurer que toutes les tâches ont une chance de s'exécuter
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void FTPHTTPProxy::setup() {
  ESP_LOGI(TAG, "Initialisation du proxy FTP/HTTP");
  
  // Créer la file d'attente de transfert
  ftp_queue = xQueueCreate(5, sizeof(ftp_transfer_request_t));
  if (ftp_queue == NULL) {
    ESP_LOGE(TAG, "Impossible de créer la file d'attente FTP");
    return;
  }
  
  // Créer la tâche FTP avec une priorité plus basse que la tâche principale
  BaseType_t result = xTaskCreatePinnedToCore(
      ftp_task_function,    // Fonction de tâche
      "ftp_task",           // Nom de la tâche
      8192,                 // Taille de la pile (octets)
      this,                 // Paramètre à passer
      5,                    // Priorité (plus basse que la tâche principale)
      &ftp_task_handle,     // Handle de tâche
      1                     // Core sur lequel exécuter (core 1, laissant le core 0 pour WiFi)
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Impossible de créer la tâche FTP");
    return;
  }
  
  this->setup_http_server();
}

void FTPHTTPProxy::loop() {
  // La boucle principale n'a plus besoin de faire grand-chose
  // car le transfert FTP se fait dans une tâche séparée
  vTaskDelay(pdMS_TO_TICKS(1)); // Céder le CPU régulièrement
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

  // Configuration du socket pour être plus robuste
  int flag = 1;
  setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
  
  // Augmenter la taille du buffer de réception
  int rcvbuf = 16384;
  setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  
  // Configuration du timeout pour éviter les blocages
  struct timeval tv;
  tv.tv_sec = 5;  // 5 secondes timeout
  tv.tv_usec = 0;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(21);
  server_addr.sin_addr.s_addr = *((unsigned long *)ftp_host->h_addr);

  // Essayer de connecter en mode non-bloquant
  fcntl(sock_, F_SETFL, O_NONBLOCK);
  int connect_result = ::connect(sock_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  
  if (connect_result < 0) {
    if (errno != EINPROGRESS) {
      ESP_LOGE(TAG, "Échec de connexion FTP : %d", errno);
      ::close(sock_);
      sock_ = -1;
      return false;
    }
    
    // Attendre que la connexion soit établie
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(sock_, &write_fds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    
    if (select(sock_ + 1, NULL, &write_fds, NULL, &tv) <= 0) {
      ESP_LOGE(TAG, "Timeout de connexion FTP");
      ::close(sock_);
      sock_ = -1;
      return false;
    }
  }
  
  // Revenir en mode bloquant
  int flags = fcntl(sock_, F_GETFL, 0);
  fcntl(sock_, F_SETFL, flags & ~O_NONBLOCK);

  // Vérifier le message de bienvenue
  char buffer[256];
  int bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "220 ")) {
    ESP_LOGE(TAG, "Message de bienvenue FTP non reçu");
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  buffer[bytes_received] = '\0';
  ESP_LOGD(TAG, "Message de bienvenue FTP: %s", buffer);

  // Céder le CPU entre les opérations
  vTaskDelay(pdMS_TO_TICKS(5));

  // Authentification
  snprintf(buffer, sizeof(buffer), "USER %s\r\n", username_.c_str());
  send(sock_, buffer, strlen(buffer), 0);
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0) {
    ESP_LOGE(TAG, "Échec de réception USER");
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  buffer[bytes_received] = '\0';
  ESP_LOGD(TAG, "Réponse USER: %s", buffer);

  // Céder le CPU
  vTaskDelay(pdMS_TO_TICKS(5));

  snprintf(buffer, sizeof(buffer), "PASS %s\r\n", password_.c_str());
  send(sock_, buffer, strlen(buffer), 0);
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "230 ")) {
    ESP_LOGE(TAG, "Échec d'authentification FTP");
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  buffer[bytes_received] = '\0';
  ESP_LOGD(TAG, "Réponse PASS: %s", buffer);

  // Céder le CPU
  vTaskDelay(pdMS_TO_TICKS(5));

  // Mode binaire
  send(sock_, "TYPE I\r\n", 8, 0);
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "200 ")) {
    ESP_LOGE(TAG, "Échec de configuration du mode binaire");
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  buffer[bytes_received] = '\0';
  ESP_LOGD(TAG, "Réponse TYPE I: %s", buffer);

  return true;
}

bool FTPHTTPProxy::download_file(const std::string &remote_path, httpd_req_t *req) {
  // Cette fonction est maintenant un wrapper qui ajoute la demande à la file d'attente
  // et attend la fin du traitement
  
  // Créer une demande de transfert
  ftp_transfer_request_t request;
  request.remote_path = remote_path;
  request.req = req;
  request.completed = false;
  request.success = false;
  
  ESP_LOGI(TAG, "Ajout de la demande de transfert pour %s à la file d'attente", remote_path.c_str());
  
  // Envoyer la demande à la tâche FTP
  if (xQueueSend(ftp_queue, &request, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Impossible d'envoyer la demande à la tâche FTP (file d'attente pleine ?)");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Serveur occupé, réessayez plus tard");
    return false;
  }
  
  // Dans cette implémentation simple, nous attendons ici
  // Dans une implémentation plus avancée, on utiliserait un système de notification ou de callback
  uint32_t timeout_count = 0;
  while (!request.completed && timeout_count < 300) { // Timeout de 30 secondes max
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout_count++;
  }
  
  if (!request.completed) {
    ESP_LOGE(TAG, "Timeout en attendant la fin du transfert");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Délai d'attente dépassé");
    return false;
  }
  
  return request.success;
}

bool FTPHTTPProxy::process_ftp_transfer(const std::string &remote_path, httpd_req_t *req) {
  // Déclarations en haut pour éviter les goto cross-initialization
  int data_sock = -1;
  bool success = false;
  char *pasv_start = nullptr;
  int data_port = 0;
  int ip[4], port[2]; 
  char buffer[4096]; // Tampon de taille réduite pour un meilleur contrôle de la mémoire
  int bytes_received;
  int flag = 1;
  int rcvbuf = 16384;
  uint32_t last_yield_time = 0;
  const uint32_t YIELD_INTERVAL_MS = 10; // Yield every 10ms

  // Connexion au serveur FTP
  if (!connect_to_ftp()) {
    ESP_LOGE(TAG, "Échec de connexion FTP");
    goto error;
  }

  // Permettre au système de respirer
  vTaskDelay(pdMS_TO_TICKS(5));

  // Mode passif
  send(sock_, "PASV\r\n", 6, 0);
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "227 ")) {
    ESP_LOGE(TAG, "Erreur en mode passif");
    goto error;
  }
  buffer[bytes_received] = '\0';
  ESP_LOGD(TAG, "Réponse PASV: %s", buffer);

  // Permettre au système de respirer
  vTaskDelay(pdMS_TO_TICKS(5));

  // Extraction des données de connexion
  pasv_start = strchr(buffer, '(');
  if (!pasv_start) {
    ESP_LOGE(TAG, "Format PASV incorrect");
    goto error;
  }

  sscanf(pasv_start, "(%d,%d,%d,%d,%d,%d)", &ip[0], &ip[1], &ip[2], &ip[3], &port[0], &port[1]);
  data_port = port[0] * 256 + port[1];
  ESP_LOGD(TAG, "Port de données: %d", data_port);

  // Création du socket de données
  data_sock = ::socket(AF_INET, SOCK_STREAM, 0);
  if (data_sock < 0) {
    ESP_LOGE(TAG, "Échec de création du socket de données");
    goto error;
  }

  // Configuration du socket de données pour être plus robuste
  setsockopt(data_sock, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
  
  // Augmenter la taille du buffer de réception pour le socket de données
  setsockopt(data_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  // Configuration en non-bloquant pour le socket de données
  fcntl(data_sock, F_SETFL, O_NONBLOCK);

  struct sockaddr_in data_addr;
  memset(&data_addr, 0, sizeof(data_addr));
  data_addr.sin_family = AF_INET;
  data_addr.sin_port = htons(data_port);
  data_addr.sin_addr.s_addr = htonl(
      (ip[0] << 24) | (ip[1] << 16) | (ip[2] << 8) | ip[3]
  );

  if (::connect(data_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) != 0) {
    // Vérifier si la connexion est en cours (EINPROGRESS)
    if (errno != EINPROGRESS) {
      ESP_LOGE(TAG, "Échec de connexion au port de données: %d", errno);
      goto error;
    }
    
    // Attendez que la connexion soit établie
    fd_set write_fds;
    struct timeval timeout;
    FD_ZERO(&write_fds);
    FD_SET(data_sock, &write_fds);
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    
    if (select(data_sock + 1, NULL, &write_fds, NULL, &timeout) <= 0) {
      ESP_LOGE(TAG, "Timeout de connexion au port de données");
      goto error;
    }
  }

  // Permettre au système de respirer
  vTaskDelay(pdMS_TO_TICKS(5));

  // Revenir en mode bloquant
  int flags = fcntl(data_sock, F_GETFL, 0);
  fcntl(data_sock, F_SETFL, flags & ~O_NONBLOCK);

  // Envoi de la commande RETR
  snprintf(buffer, sizeof(buffer), "RETR %s\r\n", remote_path.c_str());
  ESP_LOGD(TAG, "Envoi de la commande: %s", buffer);
  send(sock_, buffer, strlen(buffer), 0);

  // Vérification de la réponse 150
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received <= 0 || !strstr(buffer, "150 ")) {
    ESP_LOGE(TAG, "Fichier non trouvé ou inaccessible");
    goto error;
  }
  buffer[bytes_received] = '\0';
  ESP_LOGD(TAG, "Réponse RETR: %s", buffer);

  // Permettre au système de respirer
  vTaskDelay(pdMS_TO_TICKS(5));

  // Définir un délai pour le socket de données afin d'éviter le blocage
  struct timeval tv;
  tv.tv_sec = 1;  // 1 seconde timeout
  tv.tv_usec = 0;
  setsockopt(data_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Transfert en streaming avec un buffer plus petit
  last_yield_time = esp_timer_get_time() / 1000; // Convertir en ms
  
  while (true) {
    // Vérifier si nous devons céder le CPU
    uint32_t current_time = esp_timer_get_time() / 1000; // Convertir en ms
    if (current_time - last_yield_time >= YIELD_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(1)); // Céder le CPU
      last_yield_time = current_time;
    }
    
    bytes_received = recv(data_sock, buffer, sizeof(buffer), 0);
    if (bytes_received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Timeout, cédez le CPU et réessayez
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }
      ESP_LOGE(TAG, "Erreur de réception des données: %d", errno);
      break;
    } else if (bytes_received == 0) {
      // Fin du fichier
      break;
    }

    esp_err_t err = httpd_resp_send_chunk(req, buffer, bytes_received);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Échec d'envoi au client: %d", err);
      goto error;
    }
    
    // Petite pause après chaque chunk pour permettre au watchdog de respirer
    if (bytes_received >= 2048) {
      vTaskDelay(pdMS_TO_TICKS(5));
    } else {
      // Utiliser taskYIELD() pour les chunks plus petits pour céder le CPU
      // sans trop ralentir le transfert
      taskYIELD();
    }
  }

  // Fermeture du socket de données
  ::close(data_sock);
  data_sock = -1;

  // Vérification de la réponse finale 226
  bytes_received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received > 0 && strstr(buffer, "226 ")) {
    success = true;
    buffer[bytes_received] = '\0';
    ESP_LOGD(TAG, "Transfert terminé: %s", buffer);
  }

  // Fermeture des sockets
  send(sock_, "QUIT\r\n", 6, 0);
  ::close(sock_);
  sock_ = -1;

  // Envoi du chunk final
  httpd_resp_send_chunk(req, NULL, 0);
  return success;

error:
  if (data_sock != -1) ::close(data_sock);
  if (sock_ != -1) {
    send(sock_, "QUIT\r\n", 6, 0);
    ::close(sock_);
    sock_ = -1;
  }
  return false;
}

esp_err_t FTPHTTPProxy::http_req_handler(httpd_req_t *req) {
  auto *proxy = (FTPHTTPProxy *)req->user_ctx;
  std::string requested_path = req->uri;

  // Suppression du premier slash
  if (!requested_path.empty() && requested_path[0] == '/') {
    requested_path.erase(0, 1);
  }

  ESP_LOGI(TAG, "Requête reçue: %s", requested_path.c_str());

  // Obtenir l'extension du fichier pour déterminer le type MIME
  std::string extension = "";
  size_t dot_pos = requested_path.find_last_of('.');
  if (dot_pos != std::string::npos) {
    extension = requested_path.substr(dot_pos);
    ESP_LOGD(TAG, "Extension détectée: %s", extension.c_str());
  }

  // Extraire le nom du fichier de requested_path pour l'en-tête Content-Disposition
  std::string filename = requested_path;
  size_t slash_pos = requested_path.find_last_of('/');
  if (slash_pos != std::string::npos) {
    filename = requested_path.substr(slash_pos + 1);
  }

  // Définir les types MIME et headers selon le type de fichier
  if (extension == ".mp3") {
    httpd_resp_set_type(req, "application/octet-stream");
    std::string header = "attachment; filename=\"" + filename + "\"";
    httpd_resp_set_hdr(req, "Content-Disposition", header.c_str());
    ESP_LOGD(TAG, "Configuré pour téléchargement MP3");
  } else if (extension == ".wav") {
    httpd_resp_set_type(req, "application/octet-stream");
    std::string header = "attachment; filename=\"" + filename + "\"";
    httpd_resp_set_hdr(req, "Content-Disposition", header.c_str());
    ESP_LOGD(TAG, "Configuré pour téléchargement WAV");
  } else if (extension == ".ogg") {
    httpd_resp_set_type(req, "application/octet-stream");
    std::string header = "attachment; filename=\"" + filename + "\"";
    httpd_resp_set_hdr(req, "Content-Disposition", header.c_str());
    ESP_LOGD(TAG, "Configuré pour téléchargement OGG");
  } else if (extension == ".mp4") {
    httpd_resp_set_type(req, "video/mp4");
    std::string header = "attachment; filename=\"" + filename + "\"";
    httpd_resp_set_hdr(req, "Content-Disposition", header.c_str());
    ESP_LOGD(TAG, "Configuré pour téléchargement MP4");
  } else if (extension == ".pdf") {
    httpd_resp_set_type(req, "application/pdf");
  } else if (extension == ".jpg" || extension == ".jpeg") {
    httpd_resp_set_type(req, "image/jpeg");
  } else if (extension == ".png") {
    httpd_resp_set_type(req, "image/png");
  } else {
    // Type par défaut pour les fichiers inconnus
    httpd_resp_set_type(req, "application/octet-stream");
    std::string header = "attachment; filename=\"" + filename + "\"";
    httpd_resp_set_hdr(req, "Content-Disposition", header.c_str());
    ESP_LOGD(TAG, "Configuré pour téléchargement générique");
  }

  // Pour traiter les gros fichiers, on ajoute des en-têtes supplémentaires
  httpd_resp_set_hdr(req, "Accept-Ranges", "bytes");
  
  for (const auto &configured_path : proxy->remote_paths_) {
    if (requested_path == configured_path) {
      ESP_LOGI(TAG, "Téléchargement du fichier: %s", requested_path.c_str());
      if (proxy->download_file(configured_path, req)) {
        ESP_LOGI(TAG, "Téléchargement réussi");
        return ESP_OK;
      } else {
        ESP_LOGE(TAG, "Échec du téléchargement");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Échec du téléchargement");
        return ESP_FAIL;
      }
    }
  }

  ESP_LOGW(TAG, "Fichier non trouvé: %s", requested_path.c_str());
  httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Fichier non trouvé");
  return ESP_FAIL;
}

void FTPHTTPProxy::setup_http_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = local_port_;
  config.uri_match_fn = httpd_uri_match_wildcard;
  
  // Augmenter les limites pour gérer les grandes requêtes
  config.recv_wait_timeout = 20;
  config.send_wait_timeout = 20;
  config.max_uri_handlers = 8;
  config.max_resp_headers = 20;
  config.stack_size = 12288;  // Augmenter la taille de la pile

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








