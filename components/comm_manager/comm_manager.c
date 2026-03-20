
/* ============================================================
   COMM MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      comm_manager.c
   @brief     Camada de abstracção sobre o udp_manager
   @version   2.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Abstracção de alto nível sobre o udp_manager. Calcula o ETA
   automaticamente e gere a lógica de liderança MASTER.

   Fórmula de cálculo do ETA:
   ---------------------------
   Distância que o carro ainda tem a percorrer até à zona
   de detecção do poste seguinte:
     dist_restante = POSTE_DIST_M - RADAR_DETECT_M  (metros)
     velocidade_ms = speed_kmh / 3.6               (m/s)
     ETA_ms        = (dist_restante / velocidade_ms) * 1000

   Exemplo: poste a 50m, radar a 7m, carro a 50 km/h:
     dist = 50 - 7 = 43m
     vel  = 50 / 3.6 ≈ 13.9 m/s
     ETA  = 43 / 13.9 * 1000 ≈ 3094 ms

   Alterações v1.0 → v2.0:
   ------------------------
   CORRECÇÃO 1: wifi_is_connected() → wifi_manager_is_connected()
   CORRECÇÃO 2: wifi_is_ap_active() removido (sem modo AP)
   CORRECÇÃO 3: udp_manager_init() verificação de retorno bool
   CORRECÇÃO 4: neighbor_t.id (era .poste_id em v1.0)
   CORRECÇÃO 5: todas as funções udp_manager_send_* alinhadas
                com a nova API do udp_manager v3.0

============================================================ */

#include "comm_manager.h"
#include "udp_manager.h"
#include "wifi_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include <string.h>

/* Etiqueta de log deste módulo */
static const char *TAG = "COMM_MGR";

/* Estado interno — true após comm_init() com sucesso */
static bool s_ok = false;

/* ============================================================
   INICIALIZAÇÃO
============================================================ */

bool comm_init(void)
{
    ESP_LOGI(TAG, "A inicializar comunicação UDP...");

    /* CORRECÇÃO v2.0: wifi_manager_is_connected() em vez de
       wifi_is_connected() / wifi_is_ap_active() que não existem */
    if (!wifi_manager_is_connected())
    {
        ESP_LOGW(TAG, "Wi-Fi não activo — aguardar ligação antes"
                      " de chamar comm_init()");
        return false;
    }

    /* CORRECÇÃO v2.0: udp_manager_init() retorna bool */
    if (!udp_manager_init())
    {
        ESP_LOGE(TAG, "Falha ao inicializar UDP");
        return false;
    }

    s_ok = true;
    ESP_LOGI(TAG, "Comunicação pronta | pos=%d | %s",
             POST_POSITION,
             comm_is_master() ? "MASTER" : "SLAVE");
    return true;
}

/* ============================================================
   DESCOBERTA
============================================================ */

void comm_discover(void)
{
    if (!s_ok) return;
    udp_manager_discover();
}

/* ============================================================
   _calc_eta_ms
   ------------------------------------------------------------
   Calcula o tempo (ms) que o carro demora a chegar à zona
   de detecção do poste seguinte.
   Valor mínimo de segurança: 1000 ms.
============================================================ */
static uint32_t _calc_eta_ms(float speed_kmh)
{
    /* Velocidade nula ou inválida — valor seguro por omissão */
    if (speed_kmh <= 0.0f) return 5000;

    float speed_ms   = speed_kmh / 3.6f;
    float dist_m     = (float)POSTE_DIST_M - (float)RADAR_DETECT_M;

    /* Garante distância positiva mesmo com configuração atípica */
    if (dist_m <= 0.0f) dist_m = (float)POSTE_DIST_M * 0.5f;

    uint32_t eta = (uint32_t)((dist_m / speed_ms) * 1000.0f);

    /* Mínimo de segurança: 1 segundo */
    return (eta < 1000) ? 1000 : eta;
}

/* ============================================================
   ENVIO SPD COM ETA
   ------------------------------------------------------------
   Enviado ao vizinho direito quando o radar detecta um carro.
   O vizinho usa o ETA para acender no momento exacto.
============================================================ */
bool comm_send_spd(float speed)
{
    if (!s_ok) return false;

    neighbor_t *next = comm_get_neighbor_right();
    if (!next || next->status == NEIGHBOR_OFFLINE)
    {
        ESP_LOGD(TAG, "Sem vizinho direito online para SPD");
        return false;
    }

    uint32_t eta = _calc_eta_ms(speed);
    bool ok = udp_manager_send_spd(next->ip, speed,
                                    eta, POSTE_DIST_M);

    if (ok)
        ESP_LOGI(TAG, "SPD → ID=%d IP=%s "
                      "%.1fkm/h ETA=%lums",
                 next->id, next->ip,
                 speed, (unsigned long)eta);
    return ok;
}

/* ============================================================
   TC_INC — Carro a caminho (enviado imediatamente)
   ------------------------------------------------------------
   Enviado logo após detecção pelo radar, antes do SPD.
   O vizinho incrementa Tc e acende a 100% antecipadamente.
============================================================ */
bool comm_send_tc_inc(float speed)
{
    if (!s_ok) return false;

    neighbor_t *next = comm_get_neighbor_right();
    if (!next || next->status == NEIGHBOR_OFFLINE)
    {
        ESP_LOGD(TAG, "Sem vizinho direito online para TC_INC");
        return false;
    }

    bool ok = udp_manager_send_tc_inc(next->ip, speed);

    if (ok)
        ESP_LOGI(TAG, "TC_INC → ID=%d IP=%s %.1fkm/h",
                 next->id, next->ip, speed);
    return ok;
}

/* ============================================================
   NOTIFICA ANTERIOR — Carro já passou
   ------------------------------------------------------------
   Enviado quando o radar deste poste detecta um carro.
   O poste anterior decrementa o seu T (carro saiu da sua zona).
   Usa velocidade negativa como sinal de T-- no protocolo.
============================================================ */
bool comm_notify_prev_passed(float speed)
{
    if (!s_ok) return false;

    neighbor_t *prev = comm_get_neighbor_left();
    if (!prev || prev->status == NEIGHBOR_OFFLINE)
    {
        /* Primeiro poste da cadeia — sem vizinho esquerdo */
        ESP_LOGD(TAG, "Sem vizinho esquerdo online para T--");
        return false;
    }

    /* Velocidade negativa = sinal de T-- no protocolo TC_INC */
    bool ok = udp_manager_send_tc_inc(prev->ip, -speed);

    if (ok)
        ESP_LOGD(TAG, "T-- → ID=%d IP=%s (carro passou)",
                 prev->id, prev->ip);
    return ok;
}

/* ============================================================
   MASTER_CLAIM — Reclama liderança da linha
   ------------------------------------------------------------
   Enviado pelo poste com POST_POSITION=0 ao voltar online.
   O vizinho direito cede liderança e propaga em cadeia.
============================================================ */
void comm_send_master_claim(void)
{
    if (!s_ok) return;

    neighbor_t *next = comm_get_neighbor_right();
    if (!next)
    {
        ESP_LOGD(TAG, "Sem vizinho direito para MASTER_CLAIM");
        return;
    }

    udp_manager_send_master_claim(next->ip);
    ESP_LOGI(TAG, "MASTER_CLAIM → ID=%d IP=%s",
             next->id, next->ip);
}

/* ============================================================
   STATUS — Propaga estado para todos os vizinhos activos
============================================================ */
void comm_send_status(neighbor_status_t status)
{
    if (!s_ok) return;

    neighbor_t list[MAX_NEIGHBORS];
    size_t n = udp_manager_get_all_neighbors(list, MAX_NEIGHBORS);

    for (size_t i = 0; i < n; i++)
    {
        if (!list[i].active) continue;
        udp_manager_send_status(list[i].ip, status);
    }

    ESP_LOGD(TAG, "STATUS propagado a %d vizinhos", (int)n);
}

/* ============================================================
   ACESSO A VIZINHOS POR POSIÇÃO RELATIVA
============================================================ */

/* Vizinho esquerdo → posição POST_POSITION - 1 */
neighbor_t *comm_get_neighbor_left(void)
{
    /* Primeiro poste da cadeia nunca tem vizinho esquerdo */
    if (POST_POSITION == 0) return NULL;
    return udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
}

/* Vizinho direito → posição POST_POSITION + 1 */
neighbor_t *comm_get_neighbor_right(void)
{
    return udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
}

/* Todos os vizinhos activos */
size_t comm_get_neighbors(neighbor_t *list, size_t max)
{
    return udp_manager_get_all_neighbors(list, max);
}

/* ============================================================
   ESTADO DA LINHA
============================================================ */

/**
 * Este poste é MASTER quando:
 *   - POST_POSITION == 0  (início da cadeia), OU
 *   - Vizinho esquerdo está OFFLINE ou não existe
 */
bool comm_is_master(void)
{
    if (POST_POSITION == 0) return true;

    neighbor_t *left = comm_get_neighbor_left();
    if (!left) return true;

    return (left->status == NEIGHBOR_OFFLINE);
}

/* true se UDP inicializado e Wi-Fi activo */
bool comm_status_ok(void)
{
    if (!s_ok) return false;
    return wifi_manager_is_connected();
}

/* true se vizinho esquerdo está online */
bool comm_left_online(void)
{
    neighbor_t *left = comm_get_neighbor_left();
    if (!left) return false;
    return (left->status == NEIGHBOR_OK  ||
            left->status == NEIGHBOR_SAFE);
}

/* true se vizinho direito está online */
bool comm_right_online(void)
{
    neighbor_t *right = comm_get_neighbor_right();
    if (!right) return false;
    return (right->status == NEIGHBOR_OK  ||
            right->status == NEIGHBOR_SAFE);
}