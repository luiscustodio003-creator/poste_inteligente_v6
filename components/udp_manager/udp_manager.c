/* ============================================================
   UDP MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      udp_manager.c
   @brief     Descoberta automática de vizinhos via UDP broadcast
   @version   2.1
   @date      2026-03-19

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere a comunicação UDP para descoberta automática de vizinhos
   na mesma rede Wi-Fi. Envia broadcasts DISCOVER periodicamente
   e mantém tabela de vizinhos (esquerdo/direito) com timeout.

   CORRECÇÕES v2.0 → v2.1:
   ------------------------
   1. CORRECÇÃO CRÍTICA: Removidas macros locais UDP_PORT (4210),
      MAX_NEIGHBORS (2) e DISCOVER_INTERVAL_MS que divergiam dos
      valores em system_config.h (5005, 4, 5000). Este módulo
      usa agora exclusivamente os valores de system_config.h.

   2. CORRECÇÃO: udp_manager_tick() estava declarada no .h sem
      implementação. Agora implementada: pode ser chamada pelo
      código externo para forçar um ciclo de manutenção.

   3. CORRECÇÃO: NEIGHBOR_TIMEOUT_MS agora vem de system_config.h
      (era 15000 localmente vs 10000 no system_config.h).

   4. Adicionada validação do socket após criação.
============================================================ */

#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

/* Etiqueta de log deste módulo */
static const char *TAG = "UDP_MGR";

/* ============================================================
   Estado interno do módulo
============================================================ */
static int        udp_socket   = -1;
static neighbor_t neighbors[MAX_NEIGHBORS];
static uint32_t   last_broadcast = 0;   /* Timestamp último broadcast (ms) */

/* ============================================================
   _get_neighbor_index
   ------------------------------------------------------------
   Determina o índice na tabela de vizinhos com base no ID:
     0 = vizinho esquerdo (poste_id < POSTE_ID)
     1 = vizinho direito  (poste_id > POSTE_ID)
    -1 = ignorar (próprio poste ou ID inválido)
============================================================ */
static int _get_neighbor_index(int poste_id)
{
    if (poste_id < POSTE_ID) return 0; /* esquerda */
    if (poste_id > POSTE_ID) return 1; /* direita  */
    return -1;                         /* próprio poste — ignorar */
}

/* ============================================================
   _send_discover
   ------------------------------------------------------------
   Envia mensagem "DISCOVER:<POSTE_ID>" em broadcast UDP.
   O porto de destino é UDP_PORT (system_config.h).
============================================================ */
static void _send_discover(void)
{
    struct sockaddr_in dest;
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    char msg[32];
    snprintf(msg, sizeof(msg), "DISCOVER:%d", POSTE_ID);

    int r = sendto(udp_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));

    if (r < 0)
    {
        ESP_LOGW(TAG, "Falha ao enviar DISCOVER (errno=%d)", errno);
    }
    else
    {
        ESP_LOGD(TAG, "DISCOVER enviado: %s", msg);
    }
}

/* ============================================================
   _process_message
   ------------------------------------------------------------
   Processa uma mensagem UDP recebida. Actualiza a tabela de
   vizinhos se a mensagem for um DISCOVER válido de outro poste.
============================================================ */
static void _process_message(char *msg, char *ip)
{
    /* Verifica prefixo DISCOVER */
    if (strncmp(msg, "DISCOVER:", 9) != 0)
        return;

    int poste_id = atoi(msg + 9);

    int idx = _get_neighbor_index(poste_id);
    if (idx < 0)
        return; /* Próprio poste ou ID inválido — ignorar */

    /* Actualiza entrada na tabela de vizinhos */
    strncpy(neighbors[idx].ip, ip, sizeof(neighbors[idx].ip) - 1);
    neighbors[idx].ip[sizeof(neighbors[idx].ip) - 1] = '\0';
    neighbors[idx].poste_id  = poste_id;
    neighbors[idx].last_seen = (uint32_t)(esp_timer_get_time() / 1000);

    ESP_LOGI(TAG, "Vizinho %d (%s) actualizado [%s]",
             poste_id, ip,
             idx == 0 ? "ESQUERDA" : "DIREITA");
}

/* ============================================================
   _check_timeouts
   ------------------------------------------------------------
   Verifica expiração dos vizinhos. Se o tempo desde
   last_seen exceder NEIGHBOR_TIMEOUT_MS, o vizinho é removido.
============================================================ */
static void _check_timeouts(uint32_t now_ms)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (neighbors[i].ip[0] == '\0')
            continue; /* Entrada vazia — ignorar */

        if ((now_ms - neighbors[i].last_seen) > NEIGHBOR_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "Vizinho expirado: ID=%d IP=%s",
                     neighbors[i].poste_id, neighbors[i].ip);
            memset(&neighbors[i], 0, sizeof(neighbor_t));
        }
    }
}

/* ============================================================
   udp_task
   ------------------------------------------------------------
   Task FreeRTOS que corre permanentemente:
     - Recebe mensagens UDP (recvfrom com timeout implícito)
     - Envia DISCOVER periodicamente
     - Verifica expiração de vizinhos
============================================================ */
static void udp_task(void *arg)
{
    char               rx_buffer[64];
    struct sockaddr_in addr;

    ESP_LOGI(TAG, "Task UDP iniciada (porto %d)", UDP_PORT);

    while (1)
    {
        /* --- Recepção --- */
        socklen_t addr_len = sizeof(addr);
        int r = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer) - 1,
                         0, (struct sockaddr *)&addr, &addr_len);

        if (r > 0)
        {
            rx_buffer[r] = '\0'; /* Terminar string */

            char ip[16];
            inet_ntoa_r(addr.sin_addr, ip, sizeof(ip));

            _process_message(rx_buffer, ip);
        }

        /* --- Broadcast periódico --- */
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if ((now - last_broadcast) >= DISCOVER_INTERVAL_MS)
        {
            _send_discover();
            last_broadcast = now;
        }

        /* --- Verificação de timeouts --- */
        _check_timeouts(now);

        /* Cede CPU por 100 ms */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   udp_manager_init
   ------------------------------------------------------------
   Inicializa o socket UDP, liga-o ao porto UDP_PORT e
   arranca a task de gestão UDP.
   Deve ser chamada após o Wi-Fi estar ligado (IP obtido).
============================================================ */
void udp_manager_init(void)
{
    memset(neighbors, 0, sizeof(neighbors));
    last_broadcast = 0;

    /* Cria socket UDP */
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0)
    {
        ESP_LOGE(TAG, "Falha ao criar socket UDP (errno=%d)", errno);
        return;
    }

    /* Activa envio de broadcast */
    int broadcast = 1;
    setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST,
               &broadcast, sizeof(broadcast));

    /* Define timeout de recepção para não bloquear indefinidamente */
    struct timeval timeout = { .tv_sec = 0, .tv_usec = 100000 }; /* 100 ms */
    setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO,
               &timeout, sizeof(timeout));

    /* Liga ao porto de escuta */
    struct sockaddr_in bind_addr;
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udp_socket, (struct sockaddr *)&bind_addr,
             sizeof(bind_addr)) < 0)
    {
        ESP_LOGE(TAG, "Falha ao fazer bind no porto %d (errno=%d)",
                 UDP_PORT, errno);
        close(udp_socket);
        udp_socket = -1;
        return;
    }

    /* Arranca task de gestão UDP */
    xTaskCreate(udp_task, "udp_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "UDP Manager iniciado no porto %d", UDP_PORT);
}

/* ============================================================
   udp_manager_tick
   ------------------------------------------------------------
   Processamento manual de um ciclo de manutenção UDP.
   Útil se o código externo preferir polling em vez de task.
   A task interna já faz este ciclo — uso é opcional.
============================================================ */
void udp_manager_tick(uint32_t ms)
{
    /* Parâmetro ms reservado para uso futuro (ex: acumulador) */
    (void)ms;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    if ((now - last_broadcast) >= DISCOVER_INTERVAL_MS)
    {
        _send_discover();
        last_broadcast = now;
    }

    _check_timeouts(now);
}

/* ============================================================
   udp_manager_get_neighbors
   ------------------------------------------------------------
   Preenche nebL e nebR com os IPs dos vizinhos conhecidos.
   Se não houver vizinho, preenche com "---".
   Buffers devem ter pelo menos 16 bytes.
============================================================ */
void udp_manager_get_neighbors(char *nebL, char *nebR)
{
    if (!nebL || !nebR)
        return;

    strncpy(nebL,
            neighbors[0].ip[0] ? neighbors[0].ip : "---", 16);
    nebL[15] = '\0';

    strncpy(nebR,
            neighbors[1].ip[0] ? neighbors[1].ip : "---", 16);
    nebR[15] = '\0';
}