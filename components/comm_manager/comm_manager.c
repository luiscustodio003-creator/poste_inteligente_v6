/* ============================================================
   COMM MANAGER — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Abstraccao de alto nivel sobre o udp_manager.
   Calcula ETA automaticamente e gere a logica de MASTER.
============================================================ */

#include "comm_manager.h"
#include "wifi_manager.h"
#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "COMM_MGR";
static bool s_ok = false;

/* ===========================================================
   INICIALIZACAO
   =========================================================== */

bool comm_init(void)
{
    ESP_LOGI(TAG, "Inicializar comunicacao UDP");

    /* Wi-Fi deve estar activo antes de criar o socket */
    if (!wifi_is_connected() && !wifi_is_ap_active())
    {
        ESP_LOGW(TAG, "Wi-Fi nao activo -- aguardar");
        return false;
    }

    if (!udp_manager_init())
    {
        ESP_LOGE(TAG, "Falha iniciar UDP");
        return false;
    }

    s_ok = true;
    ESP_LOGI(TAG, "Comunicacao pronta");
    return true;
}

/* ===========================================================
   DISCOVER
   =========================================================== */

void comm_discover(void)
{
    if (!s_ok) return;
    udp_manager_discover();
}

/* ===========================================================
   CALCULO DE ETA
   -----------------------------------------------------------
   ETA = tempo que o carro demora a chegar a zona do radar
   do proximo poste, a partir do momento de deteccao.
   Formula: (POSTE_DIST_M - RADAR_DETECT_M) / velocidade_ms
   =========================================================== */

static uint32_t calc_eta_ms(float speed_kmh)
{
    if (speed_kmh <= 0.0f) return 5000; /* Valor por defeito seguro */
    float speed_ms = speed_kmh / 3.6f;
    float dist     = (float)POSTE_DIST_M - (float)RADAR_DETECT_M;
    return (uint32_t)((dist / speed_ms) * 1000.0f);
}

/* ===========================================================
   ENVIO SPD COM ETA
   -----------------------------------------------------------
   Enviado para o vizinho direito quando o radar deteta carro.
   O vizinho usa o ETA para acender no momento exacto.
   =========================================================== */

bool comm_send_spd(float speed)
{
    if (!s_ok) return false;

    neighbor_t *next = comm_get_neighbor_right();
    if (!next || next->status == NEIGHBOR_OFFLINE)
    {
        ESP_LOGD(TAG, "Sem vizinho direito para SPD");
        return false;
    }

    uint32_t eta = calc_eta_ms(speed);
    return udp_manager_send_spd(next->ip, speed, eta, POSTE_DIST_M);
}

/* ===========================================================
   TC_INC — Carro a caminho (enviado imediatamente)
   -----------------------------------------------------------
   Enviado logo apos deteccao pelo radar, antes do SPD.
   O vizinho incrementa Tc e acende a 100%.
   =========================================================== */

bool comm_send_tc_inc(float speed)
{
    if (!s_ok) return false;

    neighbor_t *next = comm_get_neighbor_right();
    if (!next || next->status == NEIGHBOR_OFFLINE)
        return false;

    bool ok = udp_manager_send_tc_inc(next->ip, speed);
    if (ok) ESP_LOGI(TAG, "TC_INC->%s %.1fkm/h", next->ip, speed);
    return ok;
}

/* ===========================================================
   NOTIFICA ANTERIOR — Carro ja passou
   -----------------------------------------------------------
   Enviado quando o radar deste poste deteta um carro.
   O poste anterior decrementa o seu T.
   Usa STATUS com campo especial para indicar T--.
   =========================================================== */

bool comm_notify_prev_passed(float speed)
{
    if (!s_ok) return false;

    neighbor_t *prev = comm_get_neighbor_left();
    if (!prev || prev->status == NEIGHBOR_OFFLINE)
        return false;

    /* Envia TC_INC com velocidade negativa como sinal de T-- */
    bool ok = udp_manager_send_tc_inc(prev->ip, -speed);
    if (ok) ESP_LOGD(TAG, "Notificou anterior ID=%d carro passou", prev->id);
    return ok;
}

/* ===========================================================
   MASTER_CLAIM — Reclama lideranca da linha
   -----------------------------------------------------------
   Enviado pelo poste com pos=0 ao voltar online.
   O vizinho direito cede lideranca e propaga em cadeia.
   =========================================================== */

void comm_send_master_claim(void)
{
    if (!s_ok) return;

    neighbor_t *next = comm_get_neighbor_right();
    if (!next) return;

    udp_manager_send_master_claim(next->ip);
    ESP_LOGI(TAG, "MASTER_CLAIM propagado para ID=%d", next->id);
}

/* ===========================================================
   STATUS — Propaga estado para todos os vizinhos
   =========================================================== */

void comm_send_status(neighbor_status_t status)
{
    if (!s_ok) return;

    neighbor_t list[MAX_NEIGHBORS];
    size_t n = udp_manager_get_neighbors(list, MAX_NEIGHBORS);

    for (size_t i = 0; i < n; i++)
    {
        if (!list[i].active) continue;
        udp_manager_send_status(list[i].ip, status);
    }
}

/* ===========================================================
   VIZINHOS POR POSICAO
   =========================================================== */

/* Vizinho esquerdo -- pos = POST_POSITION - 1 */
neighbor_t *comm_get_neighbor_left(void)
{
    if (POST_POSITION == 0) return NULL; /* Primeiro poste nao tem esquerdo */
    return udp_manager_get_neighbor_by_pos(POST_POSITION - 1);
}

/* Vizinho direito -- pos = POST_POSITION + 1 */
neighbor_t *comm_get_neighbor_right(void)
{
    return udp_manager_get_neighbor_by_pos(POST_POSITION + 1);
}

size_t comm_get_neighbors(neighbor_t *list, size_t max)
{
    return udp_manager_get_neighbors(list, max);
}

/* ===========================================================
   ESTADO DA LINHA
   =========================================================== */

/* Poste e MASTER se:
   - nao tem vizinho esquerdo (pos=0), OU
   - vizinho esquerdo esta OFFLINE */
bool comm_is_master(void)
{
    if (POST_POSITION == 0) return true;
    neighbor_t *left = comm_get_neighbor_left();
    if (!left) return true;
    return (left->status == NEIGHBOR_OFFLINE);
}

bool comm_status_ok(void)
{
    if (!s_ok) return false;
    return (wifi_is_connected() || wifi_is_ap_active());
}

bool comm_left_online(void)
{
    neighbor_t *left = comm_get_neighbor_left();
    if (!left) return false;
    return (left->status != NEIGHBOR_OFFLINE);
}

bool comm_right_online(void)
{
    neighbor_t *right = comm_get_neighbor_right();
    if (!right) return false;
    return (right->status != NEIGHBOR_OFFLINE);
}