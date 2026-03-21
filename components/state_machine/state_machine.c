/* ============================================================
   STATE MACHINE — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      state_machine.c
   @brief     Máquina de estados do Poste Inteligente
   @version   2.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Implementação completa da lógica da cadeia de postes.
   Contadores T e Tc controlam o acendimento/apagamento.
   A luz nunca vai abaixo de LIGHT_MIN (10%).

   Cenários de falha implementados:
   ----------------------------------
   1. UDP falha           → STATE_AUTONOMO (radar local assume)
   2. Radar falha         → STATE_SAFE_MODE (50% fixo)
   3. Radar recupera      → sai de SAFE_MODE para IDLE/LIGHT_ON
   4. UDP recupera        → sai de AUTONOMO para IDLE/MASTER
   5. Viz. dir. offline   → Tc=0 imediatamente
   6. Viz. esq. offline   → promove-se a MASTER
   7. Viz. esq. stall     → T limpo após T_STUCK_TIMEOUT_MS
   8. pos=0 volta online  → MASTER_CLAIM em cadeia

   Timeouts relevantes (system_config.h):
   ----------------------------------------
   TRAFIC_TIMEOUT_MS    — espera após T=0,Tc=0 para baixar luz
   T_STUCK_TIMEOUT_MS   — limpa T se vizinho ant. ficou offline
                          (definido aqui se não existir no config)

============================================================ */

#include "state_machine.h"
#include "dali_manager.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "system_config.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
//#include "freertos/timers.h"
#include <string.h>

static const char *TAG = "FSM";

/* ============================================================
   TIMEOUT DE SEGURANÇA PARA T PRESO
   Se o vizinho anterior ficou offline enquanto T>0, os carros
   nunca vão sair via T-- — limpa T após este timeout.
   Valor: 3x TRAFIC_TIMEOUT_MS como margem de segurança.
============================================================ */
#ifndef T_STUCK_TIMEOUT_MS
#define T_STUCK_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 3)
#endif

/* ============================================================
   NÚMERO DE LEITURAS CONSECUTIVAS SEM DETECÇÃO
   para confirmar falha do radar (evita falsos positivos).
============================================================ */
#define RADAR_FAIL_COUNT    10   /* 10 × 100ms = 1s sem frame válido */
#define RADAR_OK_COUNT       3   /* 3 leituras OK para recuperar      */

/* ============================================================
   ESTADO INTERNO
============================================================ */
static system_state_t s_state          = STATE_IDLE;
static int            s_T              = 0;
static int            s_Tc             = 0;
static float          s_last_speed     = 0.0f;
static bool           s_apagar_pend    = false;
static bool           s_radar_ok       = true;
static bool           s_right_online   = true;

/* Timestamps para timeouts */
static uint64_t s_last_detect_ms       = 0;
static uint64_t s_left_offline_ms      = 0;  /* quando viz. esq. foi OFF */
static bool     s_left_was_offline     = false;

/* Contadores para detecção de falha do radar */
static int s_radar_fail_cnt            = 0;
static int s_radar_ok_cnt              = 0;

/* ============================================================
   UTILITÁRIOS
============================================================ */

static uint64_t _agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* ============================================================
   _aplicar_brilho
   ------------------------------------------------------------
   Aplica brilho conforme contadores T e Tc.
   Nunca vai abaixo de LIGHT_MIN.
============================================================ */
static void _aplicar_brilho(void)
{
    if (s_T > 0 || s_Tc > 0)
    {
        dali_set_brightness(LIGHT_MAX);
        if (s_state != STATE_SAFE_MODE &&
            s_state != STATE_AUTONOMO)
            s_state = STATE_LIGHT_ON;
    }
    else
    {
        dali_set_brightness(LIGHT_MIN);
        if (s_state == STATE_LIGHT_ON)
            s_state = STATE_IDLE;
        s_last_speed = 0.0f;
    }
}

/* ============================================================
   _agendar_apagar
   ------------------------------------------------------------
   Activa flag de apagamento pendente.
   state_machine_update() verifica o timeout antes de apagar.
============================================================ */
static void _agendar_apagar(void)
{
    if (s_T <= 0 && s_Tc <= 0)
    {
        s_apagar_pend    = true;
        s_last_detect_ms = _agora_ms();
        ESP_LOGD(TAG, "Apagar agendado em %lums",
                 (unsigned long)TRAFIC_TIMEOUT_MS);
    }
}

/* ============================================================
   state_machine_get_state_name
============================================================ */
const char *state_machine_get_state_name(void)
{
    switch (s_state)
    {
        case STATE_IDLE:      return "IDLE";
        case STATE_LIGHT_ON:  return "LIGHT ON";
        case STATE_SAFE_MODE: return "SAFE MODE";
        case STATE_MASTER:    return "MASTER";
        case STATE_AUTONOMO:  return "AUTONOMO";
        default:              return "---";
    }
}

/* ============================================================
   state_machine_init
============================================================ */
void state_machine_init(void)
{
    s_state            = STATE_IDLE;
    s_T                = 0;
    s_Tc               = 0;
    s_last_speed       = 0.0f;
    s_apagar_pend      = false;
    s_radar_ok         = true;
    s_right_online     = true;
    s_left_was_offline = false;
    s_radar_fail_cnt   = 0;
    s_radar_ok_cnt     = 0;
    s_last_detect_ms   = 0;
    s_left_offline_ms  = 0;

    dali_set_brightness(LIGHT_MIN);
    ESP_LOGI(TAG, "FSM v2.0 iniciada | IDLE | %d%%", LIGHT_MIN);
}

/* ============================================================
   sm_on_radar_detect
   ------------------------------------------------------------
   Chamado quando radar local confirma carro.
   Fluxo:
     1. Cancela apagamento pendente
     2. Tc-- (era "a caminho", agora chegou)
     3. T++ (está aqui)
     4. Acende 100%
     5. Notifica vizinho anterior: T[prev]--
     6. Envia TC_INC + SPD para vizinho direito
============================================================ */
void sm_on_radar_detect(float vel)
{
    s_apagar_pend    = false;
    s_last_detect_ms = _agora_ms();
    s_last_speed     = vel;
    s_radar_ok_cnt   = RADAR_OK_COUNT; /* confirma radar OK */

    /* Converte Tc→T: carro era "a caminho", agora chegou */
    if (s_Tc > 0) s_Tc--;
    s_T++;

    /* Acende imediatamente */
    dali_set_brightness(LIGHT_MAX);
    s_state = STATE_LIGHT_ON;

    ESP_LOGI(TAG, "Radar: %.1fkm/h | T=%d Tc=%d", vel, s_T, s_Tc);

    /* Notifica vizinho anterior: carro saiu da sua zona (T--) */
    comm_notify_prev_passed(vel);

    /* Informa vizinho direito só se estiver online */
    if (s_right_online)
    {
        comm_send_tc_inc(vel);
        comm_send_spd(vel);
    }
    else
    {
        /* CENÁRIO 3: viz. dir. offline — não envia, Tc já foi a 0 */
        ESP_LOGW(TAG, "Viz. dir. OFFLINE — TC_INC/SPD não enviados");
    }
}

/* ============================================================
   sm_on_right_neighbor_offline
   ------------------------------------------------------------
   CENÁRIO 3: Vizinho direito ficou OFFLINE.
   Tc vai a zeros — não há poste para receber a informação,
   não faz sentido manter o contador.
============================================================ */
void sm_on_right_neighbor_offline(void)
{
    if (s_right_online)
    {
        s_right_online = false;

        if (s_Tc > 0)
        {
            ESP_LOGW(TAG, "Viz. dir. OFFLINE — Tc=%d → 0", s_Tc);
            s_Tc = 0;
        }

        /* Reavalia brilho com Tc=0 */
        _agendar_apagar();
    }
}

/* ============================================================
   sm_on_right_neighbor_online
   ------------------------------------------------------------
   Vizinho direito voltou online — retoma envio normal.
============================================================ */
void sm_on_right_neighbor_online(void)
{
    if (!s_right_online)
    {
        s_right_online = true;
        ESP_LOGI(TAG, "Viz. dir. voltou online");
    }
}

/* ============================================================
   CALLBACKS UDP
============================================================ */

/* Recebeu TC_INC com vel>0 — carro a caminho: Tc++ */
void on_tc_inc_received(float speed)
{
    s_apagar_pend = false;
    s_last_speed  = speed;
    s_Tc++;

    dali_set_brightness(LIGHT_MAX);
    s_state = STATE_LIGHT_ON;

    ESP_LOGI(TAG, "TC_INC+: %.1fkm/h | T=%d Tc=%d",
             speed, s_T, s_Tc);
}

/* Recebeu TC_INC com vel<0 — carro passou: T-- */
void on_prev_passed_received(void)
{
    if (s_T > 0) s_T--;

    ESP_LOGD(TAG, "Carro passou (T--) | T=%d Tc=%d", s_T, s_Tc);
    _agendar_apagar();
}

/* Recebeu SPD — informativo, guarda velocidade */
void on_spd_received(float speed, uint32_t eta_ms)
{
    s_last_speed = speed;
    ESP_LOGD(TAG, "SPD: %.1fkm/h ETA=%lums",
             speed, (unsigned long)eta_ms);
}

/* Recebeu MASTER_CLAIM — cede liderança e propaga em cadeia */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "MASTER_CLAIM de ID=%d — cedemos liderança",
             from_id);

    /* Se eramos MASTER, voltamos a IDLE */
    if (s_state == STATE_MASTER)
        s_state = STATE_IDLE;

    /* CENÁRIO 8: propaga MASTER_CLAIM para o próximo na cadeia */
    comm_send_master_claim();
}

/* ============================================================
   sm_inject_test_car
   ------------------------------------------------------------
   CENÁRIO 8 (teste): simula detecção de carro sem hardware.
   Chamar de fsm_task com botão de teste ou via flag global.
============================================================ */
void sm_inject_test_car(float vel)
{
    ESP_LOGI(TAG, "TESTE: carro simulado %.1fkm/h", vel);
    sm_on_radar_detect(vel);
}

/* ============================================================
   _verificar_radar
   ------------------------------------------------------------
   Monitoriza saúde do radar com base em leituras consecutivas.
   USE_RADAR=0: radar sempre simulado, nunca falha.
   USE_RADAR=1: conta frames sem dados para detectar falha.
============================================================ */
static void _verificar_radar(bool teve_detecao)
{
#if USE_RADAR == 0
    /* Modo simulado — radar considerado sempre OK */
    s_radar_ok     = true;
    s_radar_ok_cnt = RADAR_OK_COUNT;
    s_radar_fail_cnt = 0;
    return;
#endif

    if (teve_detecao)
    {
        s_radar_fail_cnt = 0;
        s_radar_ok_cnt++;

        /* Recuperação: CENÁRIO 7 */
        if (!s_radar_ok && s_radar_ok_cnt >= RADAR_OK_COUNT)
        {
            s_radar_ok = true;
            ESP_LOGI(TAG, "Radar recuperado — saindo de SAFE_MODE");

            if (s_state == STATE_SAFE_MODE)
            {
                /* Volta para IDLE ou LIGHT_ON conforme contadores */
                _aplicar_brilho();
            }
        }
    }
    else
    {
        s_radar_ok_cnt = 0;
        s_radar_fail_cnt++;

        if (s_radar_ok && s_radar_fail_cnt >= RADAR_FAIL_COUNT)
        {
            s_radar_ok = false;
            ESP_LOGW(TAG, "Radar sem resposta — SAFE_MODE");
        }
    }
}

/* ============================================================
   _verificar_vizinho_esquerdo
   ------------------------------------------------------------
   CENÁRIO 5 + 6: gestão de MASTER e limpeza de T preso.
============================================================ */
static void _verificar_vizinho_esquerdo(bool comm_ok, bool is_master)
{
    bool left_online = comm_left_online();

    if (!left_online && comm_ok)
    {
        /* Vizinho esquerdo acabou de ficar offline */
        if (!s_left_was_offline)
        {
            s_left_was_offline = true;
            s_left_offline_ms  = _agora_ms();

            ESP_LOGW(TAG, "Viz. esq. OFFLINE — aguardar %lums para limpar T",
                     (unsigned long)T_STUCK_TIMEOUT_MS);
        }

        /* CENÁRIO 7: T preso — limpa após timeout de segurança */
        if (s_T > 0)
        {
            uint64_t delta = _agora_ms() - s_left_offline_ms;
            if (delta >= T_STUCK_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "T preso (%d) após viz. esq. OFFLINE"
                              " — forçar T=0", s_T);
                s_T = 0;
                _agendar_apagar();
            }
        }
    }
    else if (left_online)
    {
        /* Vizinho esquerdo voltou */
        if (s_left_was_offline)
        {
            s_left_was_offline = false;
            ESP_LOGI(TAG, "Viz. esq. voltou online");
        }
    }

    /* CENÁRIO 5: actualiza estado MASTER */
    if (is_master && s_state == STATE_IDLE)
    {
        s_state = STATE_MASTER;
        ESP_LOGI(TAG, "Promovido a MASTER (viz. esq. ausente)");
    }
    else if (!is_master && s_state == STATE_MASTER)
    {
        s_state = STATE_IDLE;
        ESP_LOGI(TAG, "MASTER cedido — volta a IDLE");
    }
}

/* ============================================================
   state_machine_update
   ------------------------------------------------------------
   Ciclo principal — chamar a 100ms pela fsm_task.

   Ordem de avaliação:
     1. Lê radar (hardware ou simulado)
     2. Verifica saúde do radar
     3. Aplica SAFE_MODE se radar falhou
     4. Verifica estado do vizinho direito (Tc=0 se offline)
     5. Verifica estado do vizinho esquerdo (MASTER, T preso)
     6. Avalia timeout de apagamento (T=0, Tc=0 + 5s)
     7. Gestão de AUTONOMO ↔ normal
     8. Actualiza badge de estado no display
============================================================ */
void state_machine_update(bool comm_ok, bool is_master)
{
    /* ----------------------------------------------------------
       1+2. Lê radar e verifica saúde
    ---------------------------------------------------------- */
    radar_data_t dados;
    bool teve_detecao = false;

#if USE_RADAR
    /* Hardware real */
    teve_detecao = radar_read_data(&dados, NULL);
    if (teve_detecao && radar_vehicle_in_range(&dados))
    {
        float vel = radar_get_closest_speed(&dados);
        sm_on_radar_detect(vel > 0 ? vel : 10.0f);
    }
#else
    /* Modo simulado — detecção só via sm_inject_test_car() */
    teve_detecao = false;
#endif

    _verificar_radar(teve_detecao);

    /* ----------------------------------------------------------
       3. SAFE_MODE: radar falhou
    ---------------------------------------------------------- */
    if (!s_radar_ok)
    {
        if (s_state != STATE_SAFE_MODE)
        {
            s_state = STATE_SAFE_MODE;
            dali_set_brightness(LIGHT_SAFE_MODE);
            ESP_LOGW(TAG, "SAFE_MODE — luz %d%%", LIGHT_SAFE_MODE);

            /* Propaga falha para vizinhos */
            comm_send_status(NEIGHBOR_SAFE);
        }
        /* Em SAFE_MODE não processa mais nada neste ciclo */
        return;
    }

    /* ----------------------------------------------------------
       4. Estado do vizinho direito
    ---------------------------------------------------------- */
    bool right_now = comm_right_online();
    if (!right_now && s_right_online)
        sm_on_right_neighbor_offline();
    else if (right_now && !s_right_online)
        sm_on_right_neighbor_online();

    /* ----------------------------------------------------------
       5. Estado do vizinho esquerdo (MASTER + T preso)
    ---------------------------------------------------------- */
    _verificar_vizinho_esquerdo(comm_ok, is_master);

    /* ----------------------------------------------------------
       6. Timeout de apagamento (T=0, Tc=0 → luz 10% após 5s)
    ---------------------------------------------------------- */
    if (s_apagar_pend && s_T <= 0 && s_Tc <= 0)
    {
        uint64_t elapsed = _agora_ms() - s_last_detect_ms;
        if (elapsed >= TRAFIC_TIMEOUT_MS)
        {
            s_apagar_pend = false;
            _aplicar_brilho();
            ESP_LOGI(TAG, "T=0 Tc=0 +%lums → 10%%",
                     (unsigned long)elapsed);

            /* Notifica vizinhos que zona está limpa */
            if (comm_ok)
                comm_send_status(NEIGHBOR_OK);
        }
    }

    /* ----------------------------------------------------------
       7. AUTONOMO: sem UDP
    ---------------------------------------------------------- */
    if (!comm_ok)
    {
        if (s_state != STATE_AUTONOMO &&
            s_state != STATE_SAFE_MODE)
        {
            /* Só entra em autónomo se não houver tráfego activo */
            if (s_T == 0 && s_Tc == 0)
            {
                s_state = STATE_AUTONOMO;
                ESP_LOGW(TAG, "Sem UDP → AUTONOMO (radar local)");
            }
        }
    }
    else
    {
        /* UDP recuperou — sai de AUTONOMO */
        if (s_state == STATE_AUTONOMO)
        {
            s_state = is_master ? STATE_MASTER : STATE_IDLE;
            _aplicar_brilho();
            ESP_LOGI(TAG, "UDP recuperado → %s",
                     state_machine_get_state_name());

            /* CENÁRIO 8: pos=0 reclama liderança ao voltar */
            if (POST_POSITION == 0)
            {
                ESP_LOGI(TAG, "pos=0 volta online → MASTER_CLAIM");
                comm_send_master_claim();
            }
        }
    }
}

/* ============================================================
   GETTERS
============================================================ */

system_state_t state_machine_get_state(void)
{
    return s_state;
}

int state_machine_get_T(void)
{
    return s_T;
}

int state_machine_get_Tc(void)
{
    return s_Tc;
}

float state_machine_get_last_speed(void)
{
    return s_last_speed;
}

bool state_machine_radar_ok(void)
{
    return s_radar_ok;
}