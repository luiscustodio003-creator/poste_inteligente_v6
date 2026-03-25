/* ============================================================
   UDP MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      udp_manager.c
   @brief     Comunicação UDP entre postes — protocolo completo
   @version   3.3
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Alterações v3.2 → v3.3:
   ------------------------
   CORRECÇÃO 4: SO_RCVTIMEO reduzido de 100ms para 10ms.
                vTaskDelay reduzido de 100ms para 10ms.

                Problema: o poste A envia TC_INC seguido de SPD
                em sequência imediata. Com timeout de 100ms, o
                poste B processava TC_INC na iteração t=0 e SPD
                na iteração t=100ms (próximo recvfrom). Isto
                atrasava o cálculo do timer de pré-acendimento
                (s_acender_em_ms) em até 100ms.

                Solução: com SO_RCVTIMEO=10ms e vTaskDelay=10ms,
                TC_INC e SPD chegam em iterações separadas por
                no máximo 10ms — negligenciável para o ETA
                (que é da ordem dos 1000-4000ms).

                Impacto: ciclo da task passa de 100ms para ~10ms.
                Consumo CPU adicional: irrelevante no ESP32 @240MHz
                dado que recvfrom bloqueia aguardando pacotes.

   Alterações v3.1 → v3.2 (mantidas):
   ------------------------------------
   CORRECÇÃO 1: STATUS recebido cria vizinho se não existir.
   CORRECÇÃO 2: DISCOVER enviado imediatamente no arranque.
   CORRECÇÃO 3: udp_manager_get_neighbors() usa position.

============================================================ */

#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "UDP_MGR";

/* ============================================================
   ESTADO INTERNO
============================================================ */
static int        s_socket      = -1;
static bool       s_iniciado    = false;
static neighbor_t s_vizinhos[MAX_NEIGHBORS];

/* CORRECÇÃO 2: inicializado a 0 menos o intervalo para enviar
   DISCOVER imediatamente na primeira iteração da task */
static uint32_t   s_ultimo_disc = 0;

/* ============================================================
   UTILITÁRIOS
============================================================ */

static uint32_t _agora_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char *_status_str(neighbor_status_t s)
{
    switch (s)
    {
        case NEIGHBOR_OK:      return "OK";
        case NEIGHBOR_OFFLINE: return "OFFLINE";
        case NEIGHBOR_FAIL:    return "FAIL";
        case NEIGHBOR_SAFE:    return "SAFE";
        case NEIGHBOR_AUTO:    return "AUTO";
        default:               return "?";
    }
}

static neighbor_status_t _str_to_status(const char *s)
{
    if (strcmp(s, "OK")   == 0) return NEIGHBOR_OK;
    if (strcmp(s, "FAIL") == 0) return NEIGHBOR_FAIL;
    if (strcmp(s, "SAFE") == 0) return NEIGHBOR_SAFE;
    if (strcmp(s, "AUTO") == 0) return NEIGHBOR_AUTO;
    return NEIGHBOR_OFFLINE;
}

/* ============================================================
   _enviar_para
============================================================ */
static bool _enviar_para(const char *ip, const char *msg)
{
    if (s_socket < 0 || !ip || !msg) return false;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    dest.sin_addr.s_addr = inet_addr(ip);

    int r = sendto(s_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));

    if (r < 0)
    {
        ESP_LOGW(TAG, "Falha envio para %s: %s (errno=%d)",
                 ip, msg, errno);
        return false;
    }

    ESP_LOGD(TAG, "-> %s : %s", ip, msg);
    return true;
}

/* ============================================================
   _encontrar_ou_criar_vizinho
============================================================ */
static neighbor_t *_encontrar_ou_criar_vizinho(const char *ip,
                                                int         id,
                                                int         position)
{
    /* Procura entrada existente pelo IP */
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_vizinhos[i].active &&
            strcmp(s_vizinhos[i].ip, ip) == 0)
        {
            /* Actualiza campos se vierem mais precisos */
            if (id >= 0)       s_vizinhos[i].id       = id;
            if (position >= 0) s_vizinhos[i].position = position;
            return &s_vizinhos[i];
        }
    }

    /* Procura entrada livre */
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_vizinhos[i].active)
        {
            memset(&s_vizinhos[i], 0, sizeof(neighbor_t));
            strncpy(s_vizinhos[i].ip, ip,
                    sizeof(s_vizinhos[i].ip) - 1);
            s_vizinhos[i].id       = id;
            s_vizinhos[i].position = position;
            s_vizinhos[i].status   = NEIGHBOR_OK;
            s_vizinhos[i].active   = true;

            ESP_LOGI(TAG, "Novo vizinho ID=%d pos=%d IP=%s",
                     id, position, ip);
            return &s_vizinhos[i];
        }
    }

    ESP_LOGW(TAG, "Tabela cheia (MAX=%d)", MAX_NEIGHBORS);
    return NULL;
}

/* ============================================================
   _processar_mensagem
============================================================ */
static void _processar_mensagem(char *msg, const char *ip_origem)
{
    if (!msg || !ip_origem) return;

    /* --- DISCOVER:<id>:<position> --- */
    if (strncmp(msg, "DISCOVER:", 9) == 0)
    {
        char *p  = msg + 9;
        int   id = atoi(p);
        if (id == POSTE_ID) return;

        /* Extrai posição — fallback para id se campo ausente */
        int   pos = id;
        char *sep = strchr(p, ':');
        if (sep) pos = atoi(sep + 1);

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip_origem,
                                                     id, pos);
        if (v)
        {
            v->last_seen    = _agora_ms();
            v->status       = NEIGHBOR_OK;
            v->active       = true;
            v->discover_ok  = true;   /* DISCOVER confirmado — timeout activo */

            ESP_LOGI(TAG, "DISCOVER ID=%d pos=%d IP=%s [%s]",
                     id, pos, ip_origem,
                     pos < POST_POSITION ? "ESQ" : "DIR");
        }

        /* Responde com DISCOVER próprio + STATUS para garantir
           que o outro poste também nos regista correctamente */
        char resp[48];
        //snprintf(resp, sizeof(resp),"STATUS:%d:OK", POSTE_ID);
        /* Depois (correcto): */
        snprintf(resp, sizeof(resp), "DISCOVER:%d:%d", POSTE_ID, POST_POSITION);
        _enviar_para(ip_origem, resp);
        return;
    }

    /* --- STATUS:<id>:<estado> --- */
    if (strncmp(msg, "STATUS:", 7) == 0)
    {
        char *p  = msg + 7;
        int   id = atoi(p);
        if (id == POSTE_ID) return;

        p = strchr(p, ':'); if (!p) return; p++;
        neighbor_status_t novo_status = _str_to_status(p);

        /* Se não conhecemos este vizinho, criamos a entrada.
           Marcamos discover_confirmado=false — o timeout não
           actua sobre vizinhos sem DISCOVER confirmado ainda. */
        neighbor_t *v = _encontrar_ou_criar_vizinho(ip_origem,
                                                     id, id);
        if (v)
        {
            neighbor_status_t ant = v->status;
            v->status             = novo_status;
            v->last_seen          = _agora_ms();

            if (ant != novo_status)
                ESP_LOGI(TAG, "STATUS ID=%d: %s->%s",
                         id,
                         _status_str(ant),
                         _status_str(novo_status));
        }
        return;
    }

    /* --- SPD:<id>:<vel>:<eta_ms>:<dist_m> --- */
    if (strncmp(msg, "SPD:", 4) == 0)
    {
        char    *p   = msg + 4;
        int      id  = atoi(p);
        if (id == POSTE_ID) return;

        float    vel  = 0.0f;
        uint32_t eta  = 0;
        uint32_t dist = 0;

        p = strchr(p, ':'); if (!p) return; p++;
        vel = strtof(p, NULL);
        p = strchr(p, ':'); if (!p) return; p++;
        eta = (uint32_t)atol(p);
        p = strchr(p, ':'); if (!p) return; p++;
        dist = (uint32_t)atol(p);
        (void)dist;  /* recebido no protocolo, ETA calculado localmente */

        ESP_LOGI(TAG, "SPD de ID=%d: %.1fkm/h ETA=%lums",
                 id, vel, (unsigned long)eta);
        on_spd_received(vel, eta);
        return;
    }

    /* --- TC_INC:<id>:<vel> --- */
    if (strncmp(msg, "TC_INC:", 7) == 0)
    {
        char *p  = msg + 7;
        int   id = atoi(p);
        if (id == POSTE_ID) return;

        p = strchr(p, ':'); if (!p) return; p++;
        float vel = strtof(p, NULL);

        if (vel >= 0.0f)
        {
            ESP_LOGI(TAG, "TC_INC+ ID=%d: %.1fkm/h (Tc++)", id, vel);
            on_tc_inc_received(vel);
        }
        else
        {
            ESP_LOGI(TAG, "TC_INC- ID=%d (carro passou, T--)", id);
            on_prev_passed_received();
        }
        return;
    }

    /* --- MASTER_CLAIM:<id> --- */
    if (strncmp(msg, "MASTER_CLAIM:", 13) == 0)
    {
        int id = atoi(msg + 13);
        if (id == POSTE_ID) return;

        ESP_LOGI(TAG, "MASTER_CLAIM de ID=%d", id);
        on_master_claim_received(id);
        return;
    }

    ESP_LOGD(TAG, "Mensagem desconhecida: %s", msg);
}

/* ============================================================
   _verificar_timeouts
============================================================ */
static void _verificar_timeouts(uint32_t agora)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        /* Só verifica vizinhos activos E com DISCOVER confirmado.
           Vizinhos criados apenas por STATUS aguardam o DISCOVER
           para terem posição correcta — não entram em timeout. */
        if (!s_vizinhos[i].active)       continue;
        if (!s_vizinhos[i].discover_ok)  continue;

        uint32_t delta = agora - s_vizinhos[i].last_seen;

        if (delta > NEIGHBOR_TIMEOUT_MS &&
            s_vizinhos[i].status != NEIGHBOR_OFFLINE)
        {
            ESP_LOGW(TAG,
                     "Vizinho ID=%d IP=%s OFFLINE (%lums sem DISCOVER)",
                     s_vizinhos[i].id,
                     s_vizinhos[i].ip,
                     (unsigned long)delta);
            s_vizinhos[i].status = NEIGHBOR_OFFLINE;
        }
    }
}

/* ============================================================
   udp_task
============================================================ */
static void udp_task(void *arg)
{
    char               rx_buf[128];
    struct sockaddr_in addr_origem;

    ESP_LOGI(TAG, "Task UDP iniciada (porto %d)", UDP_PORT);

    /* CORRECÇÃO 2: envia DISCOVER imediatamente no arranque
       sem esperar DISCOVER_INTERVAL_MS */
    udp_manager_discover();
    s_ultimo_disc = _agora_ms();

    while (1)
    {
        socklen_t addr_len = sizeof(addr_origem);
        int r = recvfrom(s_socket,
                         rx_buf, sizeof(rx_buf) - 1,
                         0,
                         (struct sockaddr *)&addr_origem,
                         &addr_len);

        if (r > 0)
        {
            rx_buf[r] = '\0';
            char ip_str[16];
            inet_ntoa_r(addr_origem.sin_addr,
                        ip_str, sizeof(ip_str));
            _processar_mensagem(rx_buf, ip_str);
        }

        uint32_t agora = _agora_ms();

        if ((agora - s_ultimo_disc) >= DISCOVER_INTERVAL_MS)
        {
            udp_manager_discover();
            s_ultimo_disc = agora;
        }

        _verificar_timeouts(agora);

        /* CORRECÇÃO v3.3: 10ms — consistente com SO_RCVTIMEO */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
   udp_manager_init
============================================================ */
bool udp_manager_init(void)
{
    if (s_iniciado)
    {
        ESP_LOGW(TAG, "Já iniciado — ignorar");
        return true;
    }

    memset(s_vizinhos, 0, sizeof(s_vizinhos));
    s_ultimo_disc = 0;

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0)
    {
        ESP_LOGE(TAG, "Falha socket (errno=%d)", errno);
        return false;
    }

    int bc = 1;
    setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST,
               &bc, sizeof(bc));

    /* CORRECÇÃO v3.3: 10ms em vez de 100ms — garante que TC_INC
       e SPD enviados em sequência são processados com diferença
       máxima de 10ms, sem atrasar o timer de pré-acendimento. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
    setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO,
               &tv, sizeof(tv));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_socket,
             (struct sockaddr *)&bind_addr,
             sizeof(bind_addr)) < 0)
    {
        ESP_LOGE(TAG, "Falha bind porto %d (errno=%d)",
                 UDP_PORT, errno);
        close(s_socket);
        s_socket = -1;
        return false;
    }

    xTaskCreate(udp_task, "udp_task", 4096, NULL, 5, NULL);

    s_iniciado = true;
    ESP_LOGI(TAG, "UDP Manager v3.3 pronto (porto %d)", UDP_PORT);
    return true;
}

/* ============================================================
   udp_manager_discover
   Formato v3.1+: DISCOVER:<id>:<position>
============================================================ */
void udp_manager_discover(void)
{
    if (s_socket < 0) return;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    char msg[32];
    snprintf(msg, sizeof(msg),
             "DISCOVER:%d:%d", POSTE_ID, POST_POSITION);

    int r = sendto(s_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));

    if (r < 0)
        ESP_LOGW(TAG, "Falha DISCOVER (errno=%d)", errno);
    else
        ESP_LOGD(TAG, "DISCOVER: %s", msg);
}

/* ============================================================
   Funções de envio
============================================================ */
bool udp_manager_send_spd(const char *ip, float speed,
                            uint32_t eta_ms, uint32_t dist_m)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "SPD:%d:%.1f:%lu:%lu",
             POSTE_ID, speed,
             (unsigned long)eta_ms, (unsigned long)dist_m);
    return _enviar_para(ip, msg);
}

bool udp_manager_send_tc_inc(const char *ip, float speed)
{
    char msg[32];
    snprintf(msg, sizeof(msg),
             "TC_INC:%d:%.1f", POSTE_ID, speed);
    return _enviar_para(ip, msg);
}

bool udp_manager_send_status(const char *ip,
                               neighbor_status_t status)
{
    char msg[32];
    snprintf(msg, sizeof(msg),
             "STATUS:%d:%s", POSTE_ID, _status_str(status));
    return _enviar_para(ip, msg);
}

bool udp_manager_send_master_claim(const char *ip)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d", POSTE_ID);
    return _enviar_para(ip, msg);
}

/* ============================================================
   udp_manager_get_neighbors — compatibilidade display
============================================================ */
void udp_manager_get_neighbors(char *nebL, char *nebR)
{
    if (!nebL || !nebR) return;

    strncpy(nebL, "---", 16);
    strncpy(nebR, "---", 16);

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_vizinhos[i].active) continue;

        if (s_vizinhos[i].position < POST_POSITION)
        {
            strncpy(nebL, s_vizinhos[i].ip, 15);
            nebL[15] = '\0';
        }
        else if (s_vizinhos[i].position > POST_POSITION)
        {
            strncpy(nebR, s_vizinhos[i].ip, 15);
            nebR[15] = '\0';
        }
    }
}

/* ============================================================
   udp_manager_get_neighbor_by_pos
============================================================ */
neighbor_t *udp_manager_get_neighbor_by_pos(int position)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_vizinhos[i].active &&
            s_vizinhos[i].position == position)
            return &s_vizinhos[i];
    }
    return NULL;
}

/* ============================================================
   udp_manager_get_all_neighbors
============================================================ */
size_t udp_manager_get_all_neighbors(neighbor_t *list, size_t max)
{
    if (!list || max == 0) return 0;
    size_t n = 0;
    for (int i = 0; i < MAX_NEIGHBORS && n < max; i++)
        if (s_vizinhos[i].active)
            list[n++] = s_vizinhos[i];
    return n;
}

/* ============================================================
   CALLBACKS WEAK
============================================================ */
__attribute__((weak))
void on_tc_inc_received(float speed)
{ ESP_LOGD(TAG, "on_tc_inc_received(%.1f) sem handler", speed); }

__attribute__((weak))
void on_prev_passed_received(void)
{ ESP_LOGD(TAG, "on_prev_passed_received() sem handler"); }

__attribute__((weak))
void on_spd_received(float speed, uint32_t eta_ms)
{ ESP_LOGD(TAG, "on_spd_received(%.1f,%lu) sem handler",
           speed, (unsigned long)eta_ms); }

__attribute__((weak))
void on_master_claim_received(int from_id)
{ ESP_LOGD(TAG, "on_master_claim_received(%d) sem handler", from_id); }