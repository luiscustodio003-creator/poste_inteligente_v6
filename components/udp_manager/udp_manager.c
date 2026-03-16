/* ============================================================
   UDP MANAGER — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Camada de transporte UDP entre postes inteligentes.
   Gere socket, tabela de vizinhos, envio e recepcao
   de todas as mensagens do protocolo.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/lwip.html
   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html
============================================================ */

#include "udp_manager.h"
#include "wifi_manager.h"
#include "post_config.h"
#include "system_config.h"

#include <string.h>
#include <stdio.h>
#include <fcntl.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "UDP_MGR";

static int        s_socket         = -1;
static neighbor_t s_neighbors[MAX_NEIGHBORS];
static size_t     s_neighbor_count = 0;

#define NVS_NS  "udp_neighbors"
#define BUF_SZ  256

/* -----------------------------------------------------------
   Retorna timestamp em ms desde arranque
   ----------------------------------------------------------- */
static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* -----------------------------------------------------------
   Procura vizinho pelo ID na tabela
   ----------------------------------------------------------- */
static int find_by_id(int id)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
        if (s_neighbors[i].active && s_neighbors[i].id == id)
            return (int)i;
    return -1;
}

/* -----------------------------------------------------------
   Procura slot livre na tabela de vizinhos
   ----------------------------------------------------------- */
static int find_free(void)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
        if (!s_neighbors[i].active)
            return (int)i;
    return -1;
}

/* -----------------------------------------------------------
   Regista ou actualiza vizinho na tabela.
   Ignora o proprio poste. Persiste na NVS.
   ----------------------------------------------------------- */
static void register_neighbor(int id, const char *name,
                               int pos, const char *ip)
{
    if (id == post_get_id()) return;

    int idx = find_by_id(id);
    if (idx < 0)
    {
        idx = find_free();
        if (idx < 0) { ESP_LOGW(TAG, "Tabela cheia -- ID=%d ignorado", id); return; }
        s_neighbor_count++;
        ESP_LOGI(TAG, "Novo vizinho | ID=%d %s IP=%s pos=%d", id, name, ip, pos);
    }
    else
    {
        ESP_LOGD(TAG, "Actualizar vizinho | ID=%d IP=%s", id, ip);
    }

    strncpy(s_neighbors[idx].ip,   ip,   MAX_IP_LEN - 1);
    strncpy(s_neighbors[idx].name, name, 31);
    s_neighbors[idx].ip[MAX_IP_LEN-1] = '\0';
    s_neighbors[idx].name[31]         = '\0';
    s_neighbors[idx].id        = id;
    s_neighbors[idx].pos       = pos;
    s_neighbors[idx].last_seen = now_ms();
    s_neighbors[idx].status    = NEIGHBOR_OK;
    s_neighbors[idx].active    = true;

    udp_manager_save_neighbors_nvs();
}

/* ===========================================================
   NVS — Persistencia da tabela de vizinhos
   =========================================================== */

void udp_manager_load_neighbors_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK)
    {
        ESP_LOGD(TAG, "Sem vizinhos na NVS");
        return;
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        char k_ip[16], k_id[16], k_pos[16], k_name[16];
        snprintf(k_ip,   sizeof(k_ip),   "n%d_ip",   i);
        snprintf(k_id,   sizeof(k_id),   "n%d_id",   i);
        snprintf(k_pos,  sizeof(k_pos),  "n%d_pos",  i);
        snprintf(k_name, sizeof(k_name), "n%d_name", i);

        char     ip[MAX_IP_LEN]={0}, name[32]={0};
        size_t   ip_len=sizeof(ip), name_len=sizeof(name);
        uint32_t id=0, pos=0;

        /* Usa sempre o handle 'h' -- corrigido bug anterior */
        if (nvs_get_str(h, k_ip,  ip,   &ip_len)  != ESP_OK) continue;
        if (nvs_get_u32(h, k_id,  &id)             != ESP_OK) continue;
        if (nvs_get_u32(h, k_pos, &pos)            != ESP_OK) continue;
        nvs_get_str(h, k_name, name, &name_len);

        strncpy(s_neighbors[i].ip,   ip,   MAX_IP_LEN-1);
        strncpy(s_neighbors[i].name, name, 31);
        s_neighbors[i].id        = (int)id;
        s_neighbors[i].pos       = (int)pos;
        s_neighbors[i].last_seen = 0;           /* Offline ate confirmar */
        s_neighbors[i].status    = NEIGHBOR_OFFLINE;
        s_neighbors[i].active    = true;
        s_neighbor_count++;
        ESP_LOGI(TAG, "NVS: vizinho ID=%d IP=%s", (int)id, ip);
    }
    nvs_close(h);
}

void udp_manager_save_neighbors_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK)
    {
        ESP_LOGW(TAG, "Falha abrir NVS para guardar");
        return;
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_neighbors[i].active) continue;
        char k_ip[16], k_id[16], k_pos[16], k_name[16];
        snprintf(k_ip,   sizeof(k_ip),   "n%d_ip",   i);
        snprintf(k_id,   sizeof(k_id),   "n%d_id",   i);
        snprintf(k_pos,  sizeof(k_pos),  "n%d_pos",  i);
        snprintf(k_name, sizeof(k_name), "n%d_name", i);
        nvs_set_str(h, k_ip,   s_neighbors[i].ip);
        nvs_set_u32(h, k_id,   (uint32_t)s_neighbors[i].id);
        nvs_set_u32(h, k_pos,  (uint32_t)s_neighbors[i].pos);
        nvs_set_str(h, k_name, s_neighbors[i].name);
    }
    nvs_commit(h);
    nvs_close(h);
}

/* ===========================================================
   INICIALIZACAO DO SOCKET UDP
   =========================================================== */

bool udp_manager_init(void)
{
    memset(s_neighbors, 0, sizeof(s_neighbors));
    s_neighbor_count = 0;

    udp_manager_load_neighbors_nvs();

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0) { ESP_LOGE(TAG, "Falha criar socket"); return false; }

    /* Activa broadcast para DISCOVER e MASTER_CLAIM */
    int bcast = 1;
    setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    /* Socket nao bloqueante -- process() usa MSG_DONTWAIT */
    int flags = fcntl(s_socket, F_GETFL, 0);
    fcntl(s_socket, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(s_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ESP_LOGE(TAG, "Bind falhou porta %d", UDP_PORT);
        close(s_socket); s_socket = -1; return false;
    }

    ESP_LOGI(TAG, "UDP pronto porta %d", UDP_PORT);
    return true;
}

/* ===========================================================
   ENVIO DE MENSAGENS
   =========================================================== */

bool udp_manager_send_to(const char *buf, size_t len, const char *ip)
{
    if (s_socket < 0 || !buf || !ip) return false;
    struct sockaddr_in d = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = inet_addr(ip)
    };
    return sendto(s_socket, buf, len, 0,
                  (struct sockaddr*)&d, sizeof(d)) >= 0;
}

/* Broadcast DISCOVER -- anuncia este poste na rede */
void udp_manager_discover(void)
{
    char buf[BUF_SZ];
    int  len = snprintf(buf, sizeof(buf), "DISCOVER:%d:%s:%d",
                        post_get_id(), post_get_name(), POST_POSITION);
    struct sockaddr_in d = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = inet_addr(NETWORK_BROADCAST)
    };
    sendto(s_socket, buf, len, 0, (struct sockaddr*)&d, sizeof(d));
    ESP_LOGD(TAG, "DISCOVER enviado");
}

/* SPD com velocidade e ETA para o proximo poste */
bool udp_manager_send_spd(const char *dest_ip, float speed,
                           uint32_t eta_ms, float dist_m)
{
    char buf[BUF_SZ];
    int  len = snprintf(buf, sizeof(buf),
                        "SPD:%d:%s:%d:%.1f:%lu:%.1f",
                        post_get_id(), post_get_name(), POST_POSITION,
                        speed, (unsigned long)eta_ms, dist_m);
    bool ok = udp_manager_send_to(buf, len, dest_ip);
    ESP_LOGI(TAG, "SPD->%s %.1fkm/h ETA:%lums",
             dest_ip, speed, (unsigned long)eta_ms);
    return ok;
}

/* TC_INC -- avisa vizinho que carro esta a caminho */
bool udp_manager_send_tc_inc(const char *dest_ip, float speed)
{
    char buf[BUF_SZ];
    int  len = snprintf(buf, sizeof(buf), "TC_INC:%d:%d:%.1f",
                        post_get_id(), POST_POSITION, speed);
    return udp_manager_send_to(buf, len, dest_ip);
}

/* ACK de confirmacao de recepcao */
bool udp_manager_send_ack(const char *dest_ip, const char *msg_type)
{
    char buf[64];
    int  len = snprintf(buf, sizeof(buf), "ACK:%d:%s",
                        post_get_id(), msg_type);
    return udp_manager_send_to(buf, len, dest_ip);
}

/* STATUS -- propaga estado actual do poste */
bool udp_manager_send_status(const char *dest_ip, neighbor_status_t st)
{
    const char *s = st==NEIGHBOR_OFFLINE   ? "OFFLINE" :
                    st==NEIGHBOR_SAFE_MODE ? "SAFE"    :
                    st==NEIGHBOR_MASTER    ? "MASTER"  : "OK";
    char buf[64];
    int  len = snprintf(buf, sizeof(buf), "STATUS:%d:%d:%s",
                        post_get_id(), POST_POSITION, s);
    return udp_manager_send_to(buf, len, dest_ip);
}

/* MASTER_CLAIM -- reclama lideranca da linha */
bool udp_manager_send_master_claim(const char *dest_ip)
{
    char buf[64];
    int  len = snprintf(buf, sizeof(buf), "MASTER_CLAIM:%d:%d",
                        post_get_id(), POST_POSITION);
    bool ok = udp_manager_send_to(buf, len, dest_ip);
    if (ok) ESP_LOGI(TAG, "MASTER_CLAIM->%s", dest_ip);
    return ok;
}

/* ===========================================================
   RECEPCAO E PROCESSAMENTO
   -----------------------------------------------------------
   Le UMA mensagem do socket (nao bloqueante MSG_DONTWAIT).
   Actualiza tabela de vizinhos e retorna tipo da mensagem.
   =========================================================== */

udp_msg_type_t udp_manager_process(spd_msg_t        *spd_out,
                                    neighbor_status_t *status_out,
                                    int               *src_id_out,
                                    int               *src_pos_out)
{
    if (s_socket < 0) return UDP_MSG_NONE;

    char               buf[BUF_SZ];
    char               src_ip[MAX_IP_LEN];
    struct sockaddr_in src_addr;
    socklen_t          alen = sizeof(src_addr);

    int len = recvfrom(s_socket, buf, sizeof(buf)-1,
                       MSG_DONTWAIT,
                       (struct sockaddr*)&src_addr, &alen);
    if (len <= 0) return UDP_MSG_NONE;

    buf[len] = '\0';
    inet_ntoa_r(src_addr.sin_addr, src_ip, MAX_IP_LEN);

    /* Ignora pacotes enviados pelo proprio poste */
    const char *own = wifi_get_ip();
    if (own && strcmp(src_ip, own) == 0) return UDP_MSG_NONE;

    ESP_LOGD(TAG, "rx[%s]: %s", src_ip, buf);

    /* --- DISCOVER --- */
    if (strncmp(buf, "DISCOVER:", 9) == 0)
    {
        int id=0, pos=0; char name[32]={0};
        if (sscanf(buf, "DISCOVER:%d:%31[^:]:%d", &id, name, &pos) == 3)
        {
            register_neighbor(id, name, pos, src_ip);
            char ack[BUF_SZ];
            snprintf(ack, sizeof(ack), "DISCOVER_ACK:%d:%s:%d:%s",
                     post_get_id(), post_get_name(), POST_POSITION,
                     own ? own : "0.0.0.0");
            udp_manager_send_to(ack, strlen(ack), src_ip);
            if (src_id_out)  *src_id_out  = id;
            if (src_pos_out) *src_pos_out = pos;
        }
        return UDP_MSG_DISCOVER;
    }

    /* --- DISCOVER_ACK --- */
    if (strncmp(buf, "DISCOVER_ACK:", 13) == 0)
    {
        int id=0, pos=0; char name[32]={0}, ip[MAX_IP_LEN]={0};
        if (sscanf(buf, "DISCOVER_ACK:%d:%31[^:]:%d:%15s",
                   &id, name, &pos, ip) == 4)
            register_neighbor(id, name, pos, ip);
        return UDP_MSG_DISCOVER;
    }

    /* --- SPD: velocidade + ETA --- */
    if (strncmp(buf, "SPD:", 4) == 0)
    {
        int id=0, pos=0; float spd=0, dist=0; uint32_t eta=0;
        char name[32]={0};
        if (sscanf(buf, "SPD:%d:%31[^:]:%d:%f:%lu:%f",
                   &id, name, &pos, &spd, &eta, &dist) >= 4)
        {
            int idx = find_by_id(id);
            if (idx >= 0)
            {
                s_neighbors[idx].last_seen   = now_ms();
                s_neighbors[idx].last_speed  = spd;
                s_neighbors[idx].last_eta_ms = eta;
                s_neighbors[idx].status      = NEIGHBOR_OK;
            }
            if (spd_out)
            {
                spd_out->id     = id;
                spd_out->pos    = pos;
                spd_out->speed  = spd;
                spd_out->eta_ms = eta;
                spd_out->dist_m = dist;
            }
            if (src_id_out)  *src_id_out  = id;
            if (src_pos_out) *src_pos_out = pos;
            udp_manager_send_ack(src_ip, "SPD");
        }
        return UDP_MSG_SPD;
    }

    /* --- TC_INC: carro a caminho, incrementar Tc --- */
    if (strncmp(buf, "TC_INC:", 7) == 0)
    {
        int id=0, pos=0; float spd=0;
        if (sscanf(buf, "TC_INC:%d:%d:%f", &id, &pos, &spd) == 3)
        {
            int idx = find_by_id(id);
            if (idx >= 0)
            {
                s_neighbors[idx].last_seen  = now_ms();
                s_neighbors[idx].last_speed = spd;
                s_neighbors[idx].status     = NEIGHBOR_OK;
            }
            if (spd_out)     spd_out->speed = spd;
            if (src_id_out)  *src_id_out    = id;
            if (src_pos_out) *src_pos_out   = pos;
        }
        return UDP_MSG_TC_INC;
    }

    /* --- ACK --- */
    if (strncmp(buf, "ACK:", 4) == 0)
    {
        int id=0; char mt[16]={0};
        if (sscanf(buf, "ACK:%d:%15s", &id, mt) == 2)
        {
            int idx = find_by_id(id);
            if (idx >= 0) { s_neighbors[idx].last_seen = now_ms();
                            s_neighbors[idx].status    = NEIGHBOR_OK; }
            if (src_id_out) *src_id_out = id;
        }
        return UDP_MSG_ACK;
    }

    /* --- STATUS --- */
    if (strncmp(buf, "STATUS:", 7) == 0)
    {
        int id=0, pos=0; char estado[16]={0};
        if (sscanf(buf, "STATUS:%d:%d:%15s", &id, &pos, estado) == 3)
        {
            int idx = find_by_id(id);
            if (idx >= 0)
            {
                s_neighbors[idx].last_seen = now_ms();
                s_neighbors[idx].status    =
                    strcmp(estado, "SAFE")   ==0 ? NEIGHBOR_SAFE_MODE :
                    strcmp(estado, "MASTER") ==0 ? NEIGHBOR_MASTER    :
                    strcmp(estado, "OFFLINE")==0 ? NEIGHBOR_OFFLINE   :
                                                   NEIGHBOR_OK;
                if (status_out)  *status_out  = s_neighbors[idx].status;
                if (src_id_out)  *src_id_out  = id;
                if (src_pos_out) *src_pos_out = pos;
            }
            udp_manager_send_ack(src_ip, "STATUS");
        }
        return UDP_MSG_STATUS;
    }

    /* --- MASTER_CLAIM: reclama lideranca da linha --- */
    if (strncmp(buf, "MASTER_CLAIM:", 13) == 0)
    {
        int id=0, pos=0;
        if (sscanf(buf, "MASTER_CLAIM:%d:%d", &id, &pos) == 2)
        {
            int idx = find_by_id(id);
            if (idx >= 0) { s_neighbors[idx].last_seen = now_ms();
                            s_neighbors[idx].status    = NEIGHBOR_MASTER; }
            if (src_id_out)  *src_id_out  = id;
            if (src_pos_out) *src_pos_out = pos;
            ESP_LOGI(TAG, "MASTER_CLAIM de ID=%d pos=%d", id, pos);
        }
        return UDP_MSG_MASTER_CLAIM;
    }

    return UDP_MSG_NONE;
}

/* ===========================================================
   TABELA DE VIZINHOS
   =========================================================== */

size_t udp_manager_get_neighbors(neighbor_t *list, size_t max)
{
    size_t n = 0;
    for (size_t i = 0; i < MAX_NEIGHBORS && n < max; i++)
        if (s_neighbors[i].active) list[n++] = s_neighbors[i];
    return n;
}

bool udp_manager_has_active_neighbors(void)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
        if (s_neighbors[i].active && s_neighbors[i].status == NEIGHBOR_OK)
            return true;
    return false;
}

size_t udp_manager_get_neighbor_count(void)
{
    return s_neighbor_count;
}

/* Retorna vizinho pela posicao na cadeia ou NULL */
neighbor_t *udp_manager_get_neighbor_by_pos(int pos)
{
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
        if (s_neighbors[i].active && s_neighbors[i].pos == pos)
            return &s_neighbors[i];
    return NULL;
}

/* Marca vizinhos sem contacto como OFFLINE */
void udp_manager_check_timeouts(void)
{
    uint64_t now = now_ms();
    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_neighbors[i].active || s_neighbors[i].last_seen == 0) continue;
        if (now - s_neighbors[i].last_seen > NEIGHBOR_TIMEOUT_MS &&
            s_neighbors[i].status == NEIGHBOR_OK)
        {
            s_neighbors[i].status = NEIGHBOR_OFFLINE;
            ESP_LOGW(TAG, "Vizinho ID=%d OFFLINE (timeout)", s_neighbors[i].id);
        }
    }
}