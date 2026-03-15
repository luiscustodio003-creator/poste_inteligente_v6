/* ============================================================
   UDP MANAGER — IMPLEMENTACAO
   ------------------------------------------------------------
   @file      udp_manager.c
   @brief     Comunicacao UDP entre postes inteligentes
   @version   4.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Camada de transporte UDP para comunicacao entre postes.
   Implementa o protocolo completo de descoberta, confirmacao
   e propagacao de estado.

   Protocolo (texto simples, separado por ':'):
   ---------------------------------------------
   DISCOVER:<id>:<n>:<pos>
   DISCOVER_ACK:<id>:<n>:<pos>:<ip>
   SPD:<id>:<n>:<pos>:<velocidade>
   ACK:<id>:<msg_type>
   STATUS:<id>:<pos>:<estado>  (OK / OFFLINE / SAFE)

   Dependencias:
   -------------
   - udp_manager.h
   - wifi_manager.h  (wifi_get_ip)
   - system_config.h
   - post_config.h   (post_get_id, post_get_name)
   - lwip/sockets, esp_timer, nvs_flash, nvs
============================================================ */

#include "udp_manager.h"
#include "wifi_manager.h"
#include "system_config.h"
#include "post_config.h"

#include <string.h>
#include <stdio.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "UDP_MGR";

/* -----------------------------------------------------------
   Estado interno
   ----------------------------------------------------------- */
static int        udp_socket      = -1;
static neighbor_t s_neighbors[MAX_NEIGHBORS];
static size_t     s_neighbor_count = 0;

#define NVS_UDP_NAMESPACE   "udp_neighbors"
#define UDP_BUF_SIZE        256

/* ===========================================================
   UTILITARIOS INTERNOS
   =========================================================== */

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static int find_neighbor_by_id(int id)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_neighbors[i].active && s_neighbors[i].id == id)
            return (int)i;
    }
    return -1;
}

static int find_free_slot(void)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_neighbors[i].active)
            return (int)i;
    }
    return -1;
}

/* Regista ou actualiza vizinho na tabela.
   Persiste na NVS apos qualquer alteracao. */
static void register_neighbor(int         id,
                               const char *name,
                               int         pos,
                               const char *ip)
{
    /* Nao registar o proprio poste */
    if (id == post_get_id()) return;

    int idx = find_neighbor_by_id(id);

    if (idx < 0)
    {
        idx = find_free_slot();
        if (idx < 0)
        {
            ESP_LOGW(TAG, "Tabela cheia -- ignorar ID=%d", id);
            return;
        }
        s_neighbor_count++;
        ESP_LOGI(TAG, "Novo vizinho | ID=%d | %s | IP=%s", id, name, ip);
    }
    else
    {
        ESP_LOGD(TAG, "Actualizar vizinho | ID=%d | IP=%s", id, ip);
    }

    strncpy(s_neighbors[idx].ip,   ip,   MAX_IP_LEN - 1);
    strncpy(s_neighbors[idx].name, name, sizeof(s_neighbors[idx].name) - 1);
    s_neighbors[idx].ip[MAX_IP_LEN - 1]                    = '\0';
    s_neighbors[idx].name[sizeof(s_neighbors[idx].name)-1] = '\0';
    s_neighbors[idx].id        = id;
    s_neighbors[idx].pos       = pos;
    s_neighbors[idx].last_seen = now_ms();
    s_neighbors[idx].status    = NEIGHBOR_OK;
    s_neighbors[idx].active    = true;

    udp_manager_save_neighbors_nvs();
}

/* ===========================================================
   NVS -- Persistencia da tabela de vizinhos
   =========================================================== */

void udp_manager_load_neighbors_nvs(void)
{
    nvs_handle_t handle;

    if (nvs_open(NVS_UDP_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGD(TAG, "Sem vizinhos guardados na NVS");
        return;
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        char key_ip[16], key_id[16], key_pos[16], key_name[16];

        snprintf(key_ip,   sizeof(key_ip),   "n%d_ip",   i);
        snprintf(key_id,   sizeof(key_id),   "n%d_id",   i);
        snprintf(key_pos,  sizeof(key_pos),  "n%d_pos",  i);
        snprintf(key_name, sizeof(key_name), "n%d_name", i);

        char     ip[MAX_IP_LEN] = {0};
        char     name[32]       = {0};
        size_t   ip_len         = sizeof(ip);
        size_t   name_len       = sizeof(name);
        uint32_t id = 0, pos = 0;

        if (nvs_get_str(handle, key_ip, ip, &ip_len) != ESP_OK) continue;
        if (nvs_get_u32(handle, key_id, &id)         != ESP_OK) continue;
        if (nvs_get_u32(handle, key_pos, &pos)        != ESP_OK) continue;
        nvs_get_str(handle, key_name, name, &name_len);

        strncpy(s_neighbors[i].ip,   ip,   MAX_IP_LEN - 1);
        strncpy(s_neighbors[i].name, name, sizeof(s_neighbors[i].name) - 1);
        s_neighbors[i].id        = (int)id;
        s_neighbors[i].pos       = (int)pos;
        s_neighbors[i].last_seen = 0;          /* Offline ate confirmar */
        s_neighbors[i].status    = NEIGHBOR_OFFLINE;
        s_neighbors[i].active    = true;
        s_neighbor_count++;

        ESP_LOGI(TAG, "NVS: vizinho ID=%d IP=%s", (int)id, ip);
    }

    nvs_close(handle);
}

void udp_manager_save_neighbors_nvs(void)
{
    nvs_handle_t handle;

    if (nvs_open(NVS_UDP_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Falha abrir NVS para guardar vizinhos");
        return;
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        char key_ip[16], key_id[16], key_pos[16], key_name[16];

        snprintf(key_ip,   sizeof(key_ip),   "n%d_ip",   i);
        snprintf(key_id,   sizeof(key_id),   "n%d_id",   i);
        snprintf(key_pos,  sizeof(key_pos),  "n%d_pos",  i);
        snprintf(key_name, sizeof(key_name), "n%d_name", i);

        if (s_neighbors[i].active)
        {
            nvs_set_str(handle, key_ip,   s_neighbors[i].ip);
            nvs_set_u32(handle, key_id,   (uint32_t)s_neighbors[i].id);
            nvs_set_u32(handle, key_pos,  (uint32_t)s_neighbors[i].pos);
            nvs_set_str(handle, key_name, s_neighbors[i].name);
        }
    }

    nvs_commit(handle);
    nvs_close(handle);
}

/* ===========================================================
   INICIALIZACAO DO SOCKET UDP
   =========================================================== */

bool udp_manager_init(void)
{
    memset(s_neighbors, 0, sizeof(s_neighbors));
    s_neighbor_count = 0;

    /* Carrega vizinhos conhecidos da NVS */
    udp_manager_load_neighbors_nvs();

    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0)
    {
        ESP_LOGE(TAG, "Falha criar socket UDP");
        return false;
    }

    /* Activa broadcast */
    int broadcast = 1;
    setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST,
               &broadcast, sizeof(broadcast));

    /* Bind na porta UDP */
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(udp_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ESP_LOGE(TAG, "Erro bind UDP porta %d", UDP_PORT);
        close(udp_socket);
        udp_socket = -1;
        return false;
    }

    ESP_LOGI(TAG, "UDP pronto na porta %d", UDP_PORT);
    return true;
}

/* ===========================================================
   ENVIO
   =========================================================== */

bool udp_manager_send_to(const char *buffer, size_t len, const char *ip)
{
    if (udp_socket < 0 || !buffer || !ip) return false;

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = inet_addr(ip)
    };

    int err = sendto(udp_socket, buffer, len, 0,
                     (struct sockaddr *)&dest, sizeof(dest));
    return (err >= 0);
}

void udp_manager_discover_neighbors(void)
{
    char msg[UDP_BUF_SIZE];
    int  len = snprintf(msg, sizeof(msg),
                        "DISCOVER:%d:%s:%d",
                        post_get_id(),
                        post_get_name(),
                        POST_POSITION);

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = inet_addr(NETWORK_BROADCAST)
    };

    sendto(udp_socket, msg, len, 0,
           (struct sockaddr *)&dest, sizeof(dest));

    ESP_LOGD(TAG, "DISCOVER enviado: %s", msg);
}

bool udp_manager_send_ack(const char *dest_ip, const char *msg_type)
{
    char msg[64];
    int  len = snprintf(msg, sizeof(msg),
                        "ACK:%d:%s",
                        post_get_id(),
                        msg_type);
    return udp_manager_send_to(msg, len, dest_ip);
}

bool udp_manager_send_status(const char *dest_ip, neighbor_status_t status)
{
    const char *status_str = "OK";
    if (status == NEIGHBOR_OFFLINE)   status_str = "OFFLINE";
    if (status == NEIGHBOR_SAFE_MODE) status_str = "SAFE";

    char msg[64];
    int  len = snprintf(msg, sizeof(msg),
                        "STATUS:%d:%d:%s",
                        post_get_id(),
                        POST_POSITION,
                        status_str);
    return udp_manager_send_to(msg, len, dest_ip);
}

void udp_build_spd(char *buffer, size_t max_len, float speed)
{
    snprintf(buffer, max_len,
             "SPD:%d:%s:%d:%.1f",
             post_get_id(),
             post_get_name(),
             POST_POSITION,
             speed);
}

/* ===========================================================
   RECEPCAO E PROCESSAMENTO
   -----------------------------------------------------------
   Processa UMA mensagem (nao bloqueante MSG_DONTWAIT).
   Trata internamente: DISCOVER, DISCOVER_ACK, ACK, STATUS.
   Retorna true apenas para SPD valido.
   =========================================================== */

bool udp_manager_process(float *speed_out, int *id_out, int *pos_out)
{
    if (udp_socket < 0) return false;

    char               buffer[UDP_BUF_SIZE];
    char               src_ip[MAX_IP_LEN];
    struct sockaddr_in src_addr;
    socklen_t          addr_len = sizeof(src_addr);

    int len = recvfrom(udp_socket,
                       buffer, sizeof(buffer) - 1,
                       MSG_DONTWAIT,
                       (struct sockaddr *)&src_addr,
                       &addr_len);

    if (len <= 0) return false;

    buffer[len] = '\0';
    inet_ntoa_r(src_addr.sin_addr, src_ip, MAX_IP_LEN);

    /* Ignora mensagens do proprio poste */
    const char *own_ip = wifi_get_ip();
    if (strcmp(src_ip, own_ip) == 0) return false;

    ESP_LOGD(TAG, "UDP rx [%s]: %s", src_ip, buffer);

    /* DISCOVER:<id>:<n>:<pos> */
    if (strncmp(buffer, "DISCOVER:", 9) == 0)
    {
        int  id = 0, pos = 0;
        char name[32] = {0};

        if (sscanf(buffer, "DISCOVER:%d:%31[^:]:%d", &id, name, &pos) == 3)
        {
            register_neighbor(id, name, pos, src_ip);

            char ack[UDP_BUF_SIZE];
            snprintf(ack, sizeof(ack),
                     "DISCOVER_ACK:%d:%s:%d:%s",
                     post_get_id(),
                     post_get_name(),
                     POST_POSITION,
                     own_ip);
            udp_manager_send_to(ack, strlen(ack), src_ip);
            ESP_LOGI(TAG, "DISCOVER de ID=%d -- respondido", id);
        }
        return false;
    }

    /* DISCOVER_ACK:<id>:<n>:<pos>:<ip> */
    if (strncmp(buffer, "DISCOVER_ACK:", 13) == 0)
    {
        int  id = 0, pos = 0;
        char name[32] = {0};
        char ip[MAX_IP_LEN] = {0};

        if (sscanf(buffer, "DISCOVER_ACK:%d:%31[^:]:%d:%15s",
                   &id, name, &pos, ip) == 4)
        {
            register_neighbor(id, name, pos, ip);
            ESP_LOGI(TAG, "DISCOVER_ACK de ID=%d IP=%s", id, ip);
        }
        return false;
    }

    /* ACK:<id>:<msg_type> */
    if (strncmp(buffer, "ACK:", 4) == 0)
    {
        int  id = 0;
        char msg_type[16] = {0};

        if (sscanf(buffer, "ACK:%d:%15s", &id, msg_type) == 2)
        {
            int idx = find_neighbor_by_id(id);
            if (idx >= 0)
            {
                s_neighbors[idx].last_seen = now_ms();
                s_neighbors[idx].status    = NEIGHBOR_OK;
            }
            ESP_LOGD(TAG, "ACK de ID=%d tipo=%s", id, msg_type);
        }
        return false;
    }

    /* STATUS:<id>:<pos>:<estado> */
    if (strncmp(buffer, "STATUS:", 7) == 0)
    {
        int  id = 0, pos = 0;
        char estado[16] = {0};

        if (sscanf(buffer, "STATUS:%d:%d:%15s", &id, &pos, estado) == 3)
        {
            int idx = find_neighbor_by_id(id);
            if (idx >= 0)
            {
                s_neighbors[idx].last_seen = now_ms();

                if (strcmp(estado, "OK") == 0)
                    s_neighbors[idx].status = NEIGHBOR_OK;
                else if (strcmp(estado, "SAFE") == 0)
                    s_neighbors[idx].status = NEIGHBOR_SAFE_MODE;
                else
                    s_neighbors[idx].status = NEIGHBOR_OFFLINE;

                ESP_LOGI(TAG, "STATUS de ID=%d: %s", id, estado);
            }
            udp_manager_send_ack(src_ip, "STATUS");
        }
        return false;
    }

    /* SPD:<id>:<n>:<pos>:<velocidade> */
    if (strncmp(buffer, "SPD:", 4) == 0)
    {
        int   id = 0, pos = 0;
        float spd = 0.0f;
        char  name[32] = {0};

        if (sscanf(buffer, "SPD:%d:%31[^:]:%d:%f",
                   &id, name, &pos, &spd) == 4)
        {
            int idx = find_neighbor_by_id(id);
            if (idx >= 0)
            {
                s_neighbors[idx].last_seen  = now_ms();
                s_neighbors[idx].last_speed = spd;
                s_neighbors[idx].status     = NEIGHBOR_OK;
            }

            if (speed_out) *speed_out = spd;
            if (id_out)    *id_out    = id;
            if (pos_out)   *pos_out   = pos;

            udp_manager_send_ack(src_ip, "SPD");

            ESP_LOGI(TAG, "SPD de ID=%d: %.1f km/h", id, spd);
            return true;
        }
    }

    return false;
}

/* ===========================================================
   TABELA DE VIZINHOS
   =========================================================== */

size_t udp_manager_get_neighbors(neighbor_t *list, size_t max)
{
    size_t count = 0;

    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_neighbors[i].active)
        {
            list[count++] = s_neighbors[i];
            if (count >= max) break;
        }
    }
    return count;
}

bool udp_manager_has_active_neighbors(void)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_neighbors[i].active &&
            s_neighbors[i].status == NEIGHBOR_OK)
            return true;
    }
    return false;
}

size_t udp_manager_get_neighbor_count(void)
{
    return s_neighbor_count;
}

void udp_manager_check_timeouts(void)
{
    uint64_t now = now_ms();

    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_neighbors[i].active)          continue;
        if (s_neighbors[i].last_seen == 0)   continue;

        uint64_t elapsed = now - s_neighbors[i].last_seen;

        if (elapsed > NEIGHBOR_TIMEOUT_MS &&
            s_neighbors[i].status == NEIGHBOR_OK)
        {
            s_neighbors[i].status = NEIGHBOR_OFFLINE;
            ESP_LOGW(TAG,
                     "Vizinho ID=%d offline (%llums sem contacto)",
                     s_neighbors[i].id,
                     (unsigned long long)elapsed);
        }
    }
}