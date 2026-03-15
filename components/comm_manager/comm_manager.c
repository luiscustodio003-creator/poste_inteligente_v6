/* ============================================================
   COMM MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      comm_manager.c
   @brief     Gestão da comunicação entre postes inteligentes
   @version   3.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alterações (v2.0 → v3.0):
   --------------------------
   1. CORRECÇÃO CRÍTICA: removido wifi_start_auto() de comm_init().
      O Wi-Fi era iniciado duas vezes (aqui e em app_main()),
      causando comportamento indefinido no driver Wi-Fi do ESP32.
      O Wi-Fi deve ser iniciado APENAS em app_main().
   2. comm_init() agora só inicializa UDP — assume que o Wi-Fi
      já está activo quando é chamado.
   3. comm_receive_vehicle() actualizado para usar
      udp_manager_process() em vez de udp_manager_receive()
      (que foi removido na v4.0 do udp_manager).
   4. Adicionado comm_send_status() — propaga estado deste
      poste para todos os vizinhos directos.
   5. Adicionado comm_discover() — envia DISCOVER broadcast.

   Descrição:
   ----------
   Camada de abstração sobre udp_manager.
   Coordena: envio de SPD, recepção de SPD, propagação de
   estado (STATUS) e descoberta de vizinhos (DISCOVER).

   Dependências:
   -------------
   - comm_manager.h
   - wifi_manager.h
   - udp_manager.h
   - system_config.h
============================================================ */

#include "comm_manager.h"
#include "wifi_manager.h"
#include "udp_manager.h"
#include "system_config.h"

#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "COMM_MGR";

static bool s_comm_ok = false;

/* ===========================================================
   INICIALIZAÇÃO
   -----------------------------------------------------------
   NOTA: O Wi-Fi deve estar activo antes de chamar comm_init().
   wifi_start_auto() deve ser chamado em app_main() antes desta
   função. Não chamar wifi_start_auto() aqui.
   =========================================================== */

bool comm_init(void)
{
    ESP_LOGI(TAG, "Inicializar sistema de comunicação UDP");

    /* Verifica que o Wi-Fi já está activo */
    if (!wifi_is_connected() && !wifi_is_ap_active())
    {
        ESP_LOGW(TAG, "Wi-Fi não activo — aguardar antes de comm_init()");
        return false;
    }

    /* Inicializa socket UDP */
    if (!udp_manager_init())
    {
        ESP_LOGE(TAG, "Falha iniciar UDP");
        return false;
    }

    s_comm_ok = true;
    ESP_LOGI(TAG, "Comunicação inicializada");
    return true;
}

/* ===========================================================
   ESTADO DA COMUNICAÇÃO
   =========================================================== */

bool comm_status_ok(void)
{
    if (!s_comm_ok) return false;
    return (wifi_is_connected() || wifi_is_ap_active());
}

/* ===========================================================
   DESCOBERTA DE VIZINHOS
   =========================================================== */

void comm_discover(void)
{
    if (!s_comm_ok) return;
    udp_manager_discover_neighbors();
}

/* ===========================================================
   ENVIAR VELOCIDADE
   -----------------------------------------------------------
   Envia SPD para todos os vizinhos activos conhecidos.
   Retorna true se pelo menos um envio foi bem sucedido.
   O ACK é processado assincronamente na udp_rx_task.
   =========================================================== */

bool comm_send_vehicle(float speed)
{
    if (!s_comm_ok) return false;

    neighbor_t neighbors[MAX_NEIGHBORS];
    size_t n = udp_manager_get_neighbors(neighbors, MAX_NEIGHBORS);

    if (n == 0)
    {
        ESP_LOGW(TAG, "Sem vizinhos — SPD não enviado");
        return false;
    }

    char packet[128];
    udp_build_spd(packet, sizeof(packet), speed);

    bool sent = false;

    for (size_t i = 0; i < n; i++)
    {
        /* Só envia para vizinhos que não estão offline */
        if (neighbors[i].status == NEIGHBOR_OFFLINE) continue;

        if (udp_manager_send_to(packet, strlen(packet), neighbors[i].ip))
        {
            ESP_LOGI(TAG, "SPD → %s (ID=%d): %.1f km/h",
                     neighbors[i].ip, neighbors[i].id, speed);
            sent = true;
        }
    }

    return sent;
}

/* ===========================================================
   RECEBER VELOCIDADE
   -----------------------------------------------------------
   Processa uma mensagem UDP recebida.
   Se for SPD, preenche os parâmetros e retorna true.
   Para DISCOVER/ACK/STATUS processa internamente e retorna false.
   =========================================================== */

bool comm_receive_vehicle(float *speed, int *post_id, int *post_pos)
{
    if (!s_comm_ok) return false;

    /* udp_manager_process() trata todos os tipos de mensagem.
       Só retorna true quando recebe um pacote SPD válido.    */
    return udp_manager_process(speed, post_id, post_pos);
}

/* ===========================================================
   PROPAGAR ESTADO
   -----------------------------------------------------------
   Envia STATUS para todos os vizinhos directos.
   Usado pela state_machine quando muda de estado.
   =========================================================== */

void comm_send_status(neighbor_status_t status)
{
    if (!s_comm_ok) return;

    neighbor_t neighbors[MAX_NEIGHBORS];
    size_t n = udp_manager_get_neighbors(neighbors, MAX_NEIGHBORS);

    for (size_t i = 0; i < n; i++)
    {
        if (!neighbors[i].active) continue;

        udp_manager_send_status(neighbors[i].ip, status);
        ESP_LOGD(TAG, "STATUS → ID=%d", neighbors[i].id);
    }
}

/* ===========================================================
   TABELA DE VIZINHOS PARA O DISPLAY
   =========================================================== */

size_t comm_get_neighbors(neighbor_t *list, size_t max)
{
    return udp_manager_get_neighbors(list, max);
}