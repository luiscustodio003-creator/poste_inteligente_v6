/* ============================================================
   STATE MACHINE — IMPLEMENTAÇÃO v2.5
   ------------------------------------------------------------
   @file      state_machine.c
   @brief     Máquina de estados do Poste Inteligente
   @version   2.5
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   FLUXO TEMPORAL CORRECTO (v2.4 → v2.5):
   ----------------------------------------
   Problema anterior (v2.4):
     O poste B recebia TC_INC e acendia IMEDIATAMENTE.
     O SPD com ETA chegava depois e era tarde demais para
     servir de agendamento — só refinava o timeout de Tc.

   Solução (v2.5):
     O poste A calcula o ETA assim que detecta o carro.
     Esse ETA (tempo até o carro chegar à zona de detecção
     do poste B) é enviado dentro do próprio TC_INC.
     O poste B recebe o ETA e agenda o acendimento para
     (ETA - MARGEM_ACENDER_MS) milissegundos no futuro,
     de modo a estar ACESO antes de o carro chegar.

   Fluxo completo correcto:
   -------------------------
     1. Poste A detecta carro a RADAR_DETECT_M metros
        → T_A++ , Tc_A-- , acende A a 100%
        → notifica poste anterior: T[prev]--
        → calcula ETA_B = tempo até carro chegar à zona B
        → envia TC_INC(vel, ETA_B) para poste B
        → envia SPD(vel, ETA_B) para poste B (redundante/confirmação)

     2. Poste B recebe TC_INC(vel, ETA_B)
        → Tc_B++
        → NÃO acende imediatamente
        → agenda timer: acender em (ETA_B - MARGEM_ACENDER_MS)
        → regista ETA_B para timeout de segurança

     3. Timer expira em poste B (ETA_B - MARGEM_ACENDER_MS)
        → acende B a 100% PRÉ-CHEGADA do carro

     4. Carro chega a poste B (radar B detecta)
        → T_B++ , Tc_B-- (carro confirmado)
        → envia TC_INC para poste C (e assim por diante)

   Margem de acendimento antecipado:
   -----------------------------------
     MARGEM_ACENDER_MS = 500 ms
     Com ETA de 3094ms (50km/h, 50m postes, 7m radar):
       B acende em 3094 - 500 = 2594ms após A detectar.
       Carro chega a B em ~3094ms → 500ms de antecedência.

   Cenários de falha (inalterados de v2.4):
   ------------------------------------------
   1. UDP falha           → STATE_AUTONOMO
   2. Radar falha         → STATE_SAFE_MODE
   3. Radar recupera      → sai de SAFE_MODE
   4. UDP recupera        → sai de AUTONOMO
   5. Viz. dir. offline   → Tc=0 imediatamente
   6. Viz. esq. offline   → promove-se a MASTER
   7. Viz. esq. stall     → T limpo após T_STUCK_TIMEOUT_MS
   8. pos=0 volta online  → MASTER_CLAIM em cadeia
   9. ETA expirou sem chegada → Tc-- (segurança)

   Dependências:
   -------------
   - dali_manager.h  : controlo PWM da luminária
   - comm_manager.h  : envio de mensagens UDP
   - radar_manager.h : leitura do sensor HLK-LD2450
   - system_config.h : TRAFIC_TIMEOUT_MS, LIGHT_*, POST_POSITION

============================================================ */

#include "state_machine.h"
#include "dali_manager.h"
#include "comm_manager.h"
#include "radar_manager.h"
#include "system_config.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "FSM";

/* ============================================================
   CONSTANTES DE TIMING
============================================================ */

/* Timeout de segurança para T preso (viz. esq. offline) */
#ifndef T_STUCK_TIMEOUT_MS
#define T_STUCK_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 3)
#endif

/* Timeout de segurança para Tc (carro não chegou) */
#ifndef TC_TIMEOUT_MS
#define TC_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 2)
#endif

/*
 * MARGEM DE ACENDIMENTO ANTECIPADO (ms)
 * -----------------------------------------
 * O poste B acende (MARGEM_ACENDER_MS) antes do ETA previsto.
 * Garante que a luz está a 100% quando o carro chega.
 * Valor: 500ms — confortável mesmo com variação de velocidade.
 * Não deve ser superior a 1/4 do ETA mínimo esperado.
 */
#ifndef MARGEM_ACENDER_MS
#define MARGEM_ACENDER_MS   500UL
#endif

/* Leituras consecutivas para confirmar falha/recuperação do radar */
#define RADAR_FAIL_COUNT    10
#define RADAR_OK_COUNT       3

/* ============================================================
   ESTADO INTERNO DA FSM
============================================================ */
static system_state_t s_state          = STATE_IDLE;
static int            s_T              = 0;
static int            s_Tc             = 0;
static float          s_last_speed     = 0.0f;
static bool           s_apagar_pend    = false;
static bool           s_radar_ok       = true;
static bool           s_right_online   = true;

/* Timestamps de controlo (ms desde boot) */
static uint64_t s_last_detect_ms       = 0;  /* última detecção local  */
static uint64_t s_left_offline_ms      = 0;  /* viz. esq. ficou offline */
static uint64_t s_tc_timeout_ms        = 0;  /* timeout segurança Tc   */
static bool     s_left_was_offline     = false;

/*
 * NOVO v2.5 — Timer de acendimento antecipado
 * ----------------------------------------------
 * Quando o poste recebe TC_INC com ETA, agenda o acendimento
 * para (agora + ETA - MARGEM_ACENDER_MS).
 * s_acender_em_ms = 0 significa sem acendimento agendado.
 */
static uint64_t s_acender_em_ms        = 0;

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
   SIMULADOR FÍSICO — USE_RADAR=0
   ------------------------------------------------------------
   1 carro de cada vez com física real.
   Ciclo: AGUARDA → EM_VIA → DETECTADO → SAIU → AGUARDA

   Velocidades rotativas: 30, 50, 80, 50 km/h.
   Passo por ciclo (100ms): vy = (vel/3.6) * 100 mm
   Detecção: quando y_mm <= RADAR_DETECT_M * 1000

   O carro entra pela extremidade máxima (RADAR_MAX_MM),
   move-se em direcção a y=0 (sensor/poste) e sai quando
   y_mm <= 0.
============================================================ */
#if USE_RADAR == 0

typedef enum {
    SIM_AGUARDA   = 0,
    SIM_EM_VIA,
    SIM_DETECTADO,
    SIM_SAIU,
} sim_estado_t;

typedef struct {
    float        y_mm;          /* distância ao sensor (mm)      */
    float        x_mm;          /* posição lateral (mm)          */
    float        vy;            /* velocidade em mm/ciclo (100ms)*/
    float        vel_kmh;       /* velocidade em km/h            */
    sim_estado_t estado;
    uint64_t     t_inicio_ms;   /* timestamp início fase actual  */
    bool         injectado;     /* detecção já disparada?        */
} sim_carro_t;

static sim_carro_t s_sim_carro = {0};

/* Intervalo entre carros: TRAFIC_TIMEOUT + 3s de margem */
#define SIM_INTERVALO_MS  ((uint64_t)(TRAFIC_TIMEOUT_MS + 3000))
/* Distância de detecção em mm */
#define SIM_ZONA_MM       ((float)(RADAR_DETECT_M * 1000))

/* Velocidades rotativas dos carros simulados */
static const float s_sim_vels[] = { 30.0f, 50.0f, 80.0f, 50.0f };
static int         s_sim_vel_idx = 0;

/* ------------------------------------------------------------
   _sim_iniciar
   Lança novo carro com velocidade da tabela rotativa.
   Posição X aleatória dentro de ±384 mm do centro.
------------------------------------------------------------ */
static void _sim_iniciar(void)
{
    float vel = s_sim_vels[s_sim_vel_idx];
    s_sim_vel_idx = (s_sim_vel_idx + 1) %
        (sizeof(s_sim_vels) / sizeof(s_sim_vels[0]));

    /* X pseudo-aleatório baseado no timestamp (±384 mm) */
    s_sim_carro.x_mm        = (float)((int32_t)
                               (_agora_ms() & 0xFF) - 128) * 3.0f;
    s_sim_carro.y_mm        = (float)RADAR_MAX_MM;
    s_sim_carro.vel_kmh     = vel;
    /* vy em mm por ciclo de 100ms:
       (vel km/h) / 3.6 = m/s → × 1000 = mm/s → × 0.1 = mm/100ms */
    s_sim_carro.vy          = (vel / 3.6f) * 100.0f;
    s_sim_carro.estado      = SIM_EM_VIA;
    s_sim_carro.injectado   = false;
    s_sim_carro.t_inicio_ms = _agora_ms();

    ESP_LOGI("SIM", "Novo carro | %.0f km/h | vy=%.0f mm/ciclo",
             vel, s_sim_carro.vy);
}

/* ------------------------------------------------------------
   _sim_update
   Avança simulação 1 passo (100ms).
   Chamado por state_machine_update() a cada ciclo da FSM.
------------------------------------------------------------ */
static void _sim_update(void)
{
    uint64_t agora = _agora_ms();

    switch (s_sim_carro.estado)
    {
        /* ---- AGUARDA: espera intervalo entre carros ---- */
        case SIM_AGUARDA:
            if (s_sim_carro.t_inicio_ms == 0)
                s_sim_carro.t_inicio_ms = agora;

            if ((agora - s_sim_carro.t_inicio_ms) >= SIM_INTERVALO_MS)
                _sim_iniciar();
            break;

        /* ---- EM_VIA / DETECTADO: move o carro ---- */
        case SIM_EM_VIA:
        case SIM_DETECTADO:
            /* Avança carro em direcção ao sensor (y decresce) */
            s_sim_carro.y_mm -= s_sim_carro.vy;

            /* Dispara detecção UMA VEZ ao entrar na zona */
            if (!s_sim_carro.injectado &&
                s_sim_carro.y_mm <= SIM_ZONA_MM)
            {
                s_sim_carro.estado    = SIM_DETECTADO;
                s_sim_carro.injectado = true;
                ESP_LOGI("SIM", "Deteccao | %.0f km/h | y=%.0f mm",
                         s_sim_carro.vel_kmh, s_sim_carro.y_mm);
                sm_on_radar_detect(s_sim_carro.vel_kmh);
            }

            /* Carro passou pelo sensor → SIM_SAIU */
            if (s_sim_carro.y_mm <= 0.0f)
            {
                s_sim_carro.y_mm        = 0.0f;
                s_sim_carro.estado      = SIM_SAIU;
                s_sim_carro.t_inicio_ms = agora;
                ESP_LOGI("SIM", "Carro saiu do alcance");
            }
            break;

        /* ---- SAIU: aguarda T=0,Tc=0 + timeout ---- */
        case SIM_SAIU:
            if (s_T == 0 && s_Tc == 0 &&
                (agora - s_sim_carro.t_inicio_ms) >=
                (uint64_t)TRAFIC_TIMEOUT_MS)
            {
                s_sim_carro.estado      = SIM_AGUARDA;
                s_sim_carro.t_inicio_ms = agora;
                ESP_LOGI("SIM", "Pronto para proximo carro");
            }
            break;
    }
}

/* ------------------------------------------------------------
   sim_get_objeto
   Exporta posição actual do carro para o canvas do display.
   Chamada pelo main.c a cada ciclo de 100ms.
   Retorna false quando não há carro visível.
------------------------------------------------------------ */
bool sim_get_objeto(float *x_mm, float *y_mm)
{
    if (s_sim_carro.estado == SIM_EM_VIA ||
        s_sim_carro.estado == SIM_DETECTADO)
    {
        if (x_mm) *x_mm = s_sim_carro.x_mm;
        if (y_mm) *y_mm = s_sim_carro.y_mm;
        return true;
    }
    return false;
}

#endif /* USE_RADAR == 0 */


/* ============================================================
   _aplicar_brilho
   ------------------------------------------------------------
   Aplica brilho conforme T e Tc actuais.
   T>0 ou Tc>0 → 100% | T=0 e Tc=0 → LIGHT_MIN
   Preserva MASTER, SAFE_MODE e AUTONOMO.
============================================================ */
static void _aplicar_brilho(void)
{
    if (s_T > 0 || s_Tc > 0)
    {
        dali_set_brightness(LIGHT_MAX);
        if (s_state != STATE_SAFE_MODE &&
            s_state != STATE_AUTONOMO  &&
            s_state != STATE_MASTER)
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
    s_tc_timeout_ms    = 0;
    s_acender_em_ms    = 0;  /* NOVO v2.5 */

    dali_set_brightness(LIGHT_MIN);
    ESP_LOGI(TAG, "FSM v2.5 iniciada | IDLE | %d%%", LIGHT_MIN);
}

/* ============================================================
   sm_on_radar_detect
   ------------------------------------------------------------
   Chamado quando o radar local confirma carro na zona.

   Fluxo v2.5:
     1. Cancela apagamento e timer de acendimento pendentes
     2. Tc-- (era "a caminho", agora chegou — confirma)
     3. T++ (está aqui)
     4. Acende 100%
     5. Notifica poste anterior: T[prev]--
     6. Calcula ETA para poste B e envia TC_INC + SPD

   NOTA sobre T:
     T conta carros PRESENTES confirmados pelo radar local.
     Decrementa via on_prev_passed_received() enviado pelo
     poste seguinte quando o seu radar confirma a chegada.
============================================================ */
void sm_on_radar_detect(float vel)
{
    /* Cancela qualquer timer de acendimento antecipado pendente */
    s_acender_em_ms  = 0;

    s_apagar_pend    = false;
    s_last_detect_ms = _agora_ms();
    s_last_speed     = vel;
    s_radar_ok_cnt   = RADAR_OK_COUNT;

    /* Converte Tc→T: carro era "a caminho", chegou */
    if (s_Tc > 0) s_Tc--;
    if (s_T < MAX_RADAR_TARGETS) s_T++;

    /* Acende — preserva MASTER */
    dali_set_brightness(LIGHT_MAX);
    if (s_state != STATE_MASTER)
        s_state = STATE_LIGHT_ON;

    ESP_LOGI(TAG, "Radar: %.1f km/h | T=%d Tc=%d", vel, s_T, s_Tc);

    /* Notifica poste anterior: carro saiu da sua zona */
    comm_notify_prev_passed(vel);

    /* Informa poste direito com velocidade e ETA calculado */
    if (s_right_online)
    {
        /*
         * Ordem correta (v2.5):
         *   1. TC_INC: aviso imediato de carro a caminho
         *      (o poste B agenda acendimento com o ETA que vem
         *       no SPD logo a seguir — latência UDP ~1ms local)
         *   2. SPD:    confirma velocidade e ETA para o poste B
         *              usar em on_spd_received()
         */
        comm_send_tc_inc(vel);
        comm_send_spd(vel);
    }
    else
    {
        ESP_LOGW(TAG, "Viz. dir. OFFLINE — TC_INC/SPD nao enviados");
    }
}

/* ============================================================
   sm_on_right_neighbor_offline
============================================================ */
void sm_on_right_neighbor_offline(void)
{
    if (s_right_online)
    {
        s_right_online  = false;
        s_acender_em_ms = 0;   /* cancela timer de acendimento */

        if (s_Tc > 0)
        {
            ESP_LOGW(TAG, "Viz. dir. OFFLINE — Tc=%d → 0", s_Tc);
            s_Tc = 0;
        }
        _agendar_apagar();
    }
}

/* ============================================================
   sm_on_right_neighbor_online
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

/* ------------------------------------------------------------
   on_tc_inc_received
   ------------------------------------------------------------
   Recebeu TC_INC(vel) do poste anterior — carro a caminho.

   Fluxo v2.5 — acendimento PRÉ-ETA:
     1. Tc++
     2. NÃO acende imediatamente
     3. Aguarda on_spd_received() que traz o ETA calculado
        pelo poste anterior (chega ~1ms depois via UDP)
     4. on_spd_received() agenda s_acender_em_ms = agora+ETA-MARGEM

   Fallback de segurança:
     Se on_spd_received() não chegar em TC_TIMEOUT_MS,
     o timeout de Tc limpa o contador (cenário de UDP perdido).
     Adicionalmente, se s_acender_em_ms não for preenchido
     em 1000ms, acende por precaução (ver state_machine_update).
------------------------------------------------------------ */
void on_tc_inc_received(float speed)
{
    s_apagar_pend = false;
    s_last_speed  = speed;
    s_Tc++;

    /*
     * NÃO acender ainda — espera o ETA do SPD para decidir
     * quando acender. O SPD chega imediatamente a seguir
     * (enviado logo após TC_INC em sm_on_radar_detect).
     *
     * Regista timestamp de chegada do TC_INC para o fallback
     * de segurança em state_machine_update().
     */
    s_last_detect_ms = _agora_ms();
    s_tc_timeout_ms  = _agora_ms() + TC_TIMEOUT_MS;

    /* Propaga TC_INC em cadeia (A→B→C→D...) */
    if (s_right_online)
        comm_send_tc_inc(speed);

    ESP_LOGI(TAG, "TC_INC recebido | %.1f km/h | T=%d Tc=%d"
                  " | aguardar SPD para ETA",
             speed, s_T, s_Tc);
}

/* ------------------------------------------------------------
   on_prev_passed_received
   Recebeu TC_INC com vel<0 — carro saiu: T--
------------------------------------------------------------ */
void on_prev_passed_received(void)
{
    if (s_T > 0) s_T--;

    ESP_LOGD(TAG, "Carro passou (T--) | T=%d Tc=%d", s_T, s_Tc);
    _agendar_apagar();
}

/* ------------------------------------------------------------
   on_spd_received
   ------------------------------------------------------------
   Recebeu SPD(vel, eta_ms) — agora calcula quando acender.

   LÓGICA CENTRAL DO PRÉ-ACENDIMENTO (v2.5):
     acender_em = agora + eta_ms - MARGEM_ACENDER_MS

   Se eta_ms <= MARGEM_ACENDER_MS o carro já está perto:
     acende imediatamente (não vale a pena agendar).

   O ETA vem calculado pelo comm_manager do poste anterior:
     ETA = (POSTE_DIST_M - RADAR_DETECT_M) / (vel/3.6) * 1000
   Exemplo: 50km/h, 50m postes, 7m radar
     ETA = (50-7) / (50/3.6) * 1000 = 43/13.9 * 1000 ≈ 3094ms
     acender_em = agora + 3094 - 500 = agora + 2594ms
     → B acende 500ms antes de o carro chegar.
------------------------------------------------------------ */
void on_spd_received(float speed, uint32_t eta_ms)
{
    s_last_speed = speed;

    if (s_Tc <= 0)
    {
        /* Tc já foi limpo (timeout ou carro já passou) — ignora */
        ESP_LOGD(TAG, "SPD ignorado (Tc=0) | %.1f km/h ETA=%lums",
                 speed, (unsigned long)eta_ms);
        return;
    }

    if (eta_ms > MARGEM_ACENDER_MS)
    {
        /*
         * Agenda acendimento antecipado:
         * B acende em (ETA - MARGEM) ms a partir de agora.
         */
        s_acender_em_ms = _agora_ms() + (uint64_t)(eta_ms - MARGEM_ACENDER_MS);

        /*
         * Timeout de segurança Tc: se o carro não chegar
         * em 2×ETA, limpa Tc (UDP seguinte pode ter perdido).
         */
        s_tc_timeout_ms = _agora_ms() + (uint64_t)(eta_ms * 2);

        ESP_LOGI(TAG, "SPD: %.1f km/h | ETA=%lums"
                      " | acender em %lums (-%lums margem)",
                 speed,
                 (unsigned long)eta_ms,
                 (unsigned long)(eta_ms - MARGEM_ACENDER_MS),
                 (unsigned long)MARGEM_ACENDER_MS);
    }
    else
    {
        /*
         * ETA muito curto — carro quase a chegar:
         * acende imediatamente sem agendar timer.
         */
        s_acender_em_ms = 0;
        dali_set_brightness(LIGHT_MAX);
        if (s_state != STATE_MASTER)
            s_state = STATE_LIGHT_ON;

        ESP_LOGI(TAG, "SPD: ETA=%lums < margem — acender IMEDIATO",
                 (unsigned long)eta_ms);
    }
}

/* ------------------------------------------------------------
   on_master_claim_received
   Recebeu MASTER_CLAIM — cede liderança e propaga em cadeia.
------------------------------------------------------------ */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "MASTER_CLAIM de ID=%d — cedemos liderança",
             from_id);

    if (s_state == STATE_MASTER)
        s_state = STATE_IDLE;

    comm_send_master_claim();
}

/* ============================================================
   sm_inject_test_car
   Injecta carro simulado manualmente (teste sem hardware).
============================================================ */
void sm_inject_test_car(float vel)
{
    ESP_LOGI(TAG, "TESTE: carro simulado %.1f km/h", vel);
    sm_on_radar_detect(vel);
}

/* ============================================================
   _verificar_radar
   ------------------------------------------------------------
   Monitoriza saúde do radar (USE_RADAR=1).
   USE_RADAR=0: radar sempre OK (simulado nunca falha).
============================================================ */
static void _verificar_radar(bool teve_detecao)
{
#if USE_RADAR == 0
    s_radar_ok       = true;
    s_radar_ok_cnt   = RADAR_OK_COUNT;
    s_radar_fail_cnt = 0;
    return;
#endif

    if (teve_detecao)
    {
        s_radar_fail_cnt = 0;
        s_radar_ok_cnt++;

        if (!s_radar_ok && s_radar_ok_cnt >= RADAR_OK_COUNT)
        {
            s_radar_ok = true;
            ESP_LOGI(TAG, "Radar recuperado — saindo de SAFE_MODE");

            if (s_state == STATE_SAFE_MODE)
                _aplicar_brilho();
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
   Gestão de MASTER e limpeza de T preso.
============================================================ */
static void _verificar_vizinho_esquerdo(bool comm_ok, bool is_master)
{
    bool left_online = comm_left_online();

    if (!left_online && comm_ok)
    {
        if (!s_left_was_offline)
        {
            s_left_was_offline = true;
            s_left_offline_ms  = _agora_ms();
            ESP_LOGW(TAG, "Viz. esq. OFFLINE — aguardar T_STUCK");
        }

        if (s_T > 0)
        {
            uint64_t delta = _agora_ms() - s_left_offline_ms;
            if (delta >= T_STUCK_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "T preso (%d) — forçar T=0", s_T);
                s_T = 0;
                _agendar_apagar();
            }
        }
    }
    else if (left_online && s_left_was_offline)
    {
        s_left_was_offline = false;
        ESP_LOGI(TAG, "Viz. esq. voltou online");
    }

    /* Actualiza estado MASTER */
    if (is_master && s_state == STATE_IDLE)
    {
        s_state = STATE_MASTER;
        ESP_LOGI(TAG, "Promovido a MASTER");
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
     3. SAFE_MODE se radar falhou
     4. Verifica vizinho direito (Tc=0 se offline)
     5. Verifica vizinho esquerdo (MASTER, T preso)
     6. NOVO v2.5: verifica timer de acendimento antecipado
     7. Timeout de apagamento (T=0,Tc=0 + TRAFIC_TIMEOUT_MS)
     8. Timeout Tc (carro não chegou)
     9. Fallback segurança: TC_INC sem SPD por >1s → acende
    10. AUTONOMO ↔ normal
============================================================ */
void state_machine_update(bool comm_ok, bool is_master)
{
    /* ----------------------------------------------------------
       1+2. Lê radar e verifica saúde
    ---------------------------------------------------------- */
    bool teve_detecao = false;

#if USE_RADAR
    radar_data_t dados;
    teve_detecao = radar_read_data(&dados, NULL);
    if (teve_detecao && radar_vehicle_in_range(&dados))
    {
        float vel = radar_get_closest_speed(&dados);
        sm_on_radar_detect(vel > 0.0f ? vel : 10.0f);
    }
#else
    _sim_update();
    teve_detecao = false; /* simulado — saúde sempre OK */
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
            s_acender_em_ms = 0;   /* cancela timer */
            ESP_LOGW(TAG, "SAFE_MODE — luz %d%%", LIGHT_SAFE_MODE);
            comm_send_status(NEIGHBOR_SAFE);
        }
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
       6. NOVO v2.5 — Timer de acendimento antecipado
       ----------------------------------------------------------
       Verifica se chegou a hora de acender antes do carro.
       Condição: s_acender_em_ms > 0 e agora >= s_acender_em_ms.
       Só actua se Tc > 0 (há carro previsto a caminho).
    ---------------------------------------------------------- */
    if (s_acender_em_ms > 0 && _agora_ms() >= s_acender_em_ms)
    {
        s_acender_em_ms = 0;  /* limpa timer — não dispara de novo */

        if (s_Tc > 0)
        {
            dali_set_brightness(LIGHT_MAX);
            if (s_state != STATE_MASTER)
                s_state = STATE_LIGHT_ON;

            ESP_LOGI(TAG, "PRE-ETA: acender %lums antes do carro"
                          " | T=%d Tc=%d",
                     (unsigned long)MARGEM_ACENDER_MS, s_T, s_Tc);
        }
    }

    /* ----------------------------------------------------------
       7. Timeout de apagamento (T=0, Tc=0 → luz 10%)
    ---------------------------------------------------------- */
    if (s_apagar_pend && s_T <= 0 && s_Tc <= 0)
    {
        uint64_t elapsed = _agora_ms() - s_last_detect_ms;
        if (elapsed >= TRAFIC_TIMEOUT_MS)
        {
            s_apagar_pend = false;
            _aplicar_brilho();
            ESP_LOGI(TAG, "T=0 Tc=0 +%lums → %d%%",
                     (unsigned long)elapsed, LIGHT_MIN);

            if (comm_ok)
                comm_send_status(NEIGHBOR_OK);
        }
    }

    /* ----------------------------------------------------------
       8. Timeout Tc — carro não chegou (UDP perdido)
    ---------------------------------------------------------- */
    if (s_Tc > 0 && s_tc_timeout_ms > 0 &&
        _agora_ms() >= s_tc_timeout_ms)
    {
        ESP_LOGW(TAG, "Tc timeout — carro nao chegou | Tc=%d→%d",
                 s_Tc, s_Tc - 1);
        s_Tc--;
        s_acender_em_ms = 0;   /* cancela timer de acendimento */
        s_tc_timeout_ms = (s_Tc > 0)
                          ? (_agora_ms() + TC_TIMEOUT_MS) : 0;
        _agendar_apagar();
    }

    /* ----------------------------------------------------------
       9. Fallback de segurança:
          TC_INC recebido há >1000ms mas SPD ainda não chegou.
          Acende por precaução para não deixar o carro no escuro.
          (UDP pode ter perdido o pacote SPD.)
    ---------------------------------------------------------- */
    if (s_Tc > 0                     &&
        s_acender_em_ms == 0         &&
        s_last_detect_ms > 0         &&
        dali_get_brightness() < LIGHT_MAX)
    {
        uint64_t espera = _agora_ms() - s_last_detect_ms;
        if (espera >= 1000ULL)
        {
            dali_set_brightness(LIGHT_MAX);
            if (s_state != STATE_MASTER)
                s_state = STATE_LIGHT_ON;

            ESP_LOGW(TAG, "Fallback: TC_INC sem SPD >1s — acender");
        }
    }

    /* ----------------------------------------------------------
       10. AUTONOMO ↔ normal
    ---------------------------------------------------------- */
    if (!comm_ok)
    {
        if (s_state != STATE_AUTONOMO &&
            s_state != STATE_SAFE_MODE)
        {
            if (s_T == 0 && s_Tc == 0)
            {
                s_state         = STATE_AUTONOMO;
                s_acender_em_ms = 0;
                ESP_LOGW(TAG, "Sem UDP → AUTONOMO");
            }
        }
    }
    else
    {
        if (s_state == STATE_AUTONOMO)
        {
            s_state = is_master ? STATE_MASTER : STATE_IDLE;
            _aplicar_brilho();
            ESP_LOGI(TAG, "UDP recuperado → %s",
                     state_machine_get_state_name());

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

system_state_t state_machine_get_state(void)   { return s_state; }
int            state_machine_get_T(void)        { return s_T; }
int            state_machine_get_Tc(void)       { return s_Tc; }
float          state_machine_get_last_speed(void) { return s_last_speed; }
bool           state_machine_radar_ok(void)     { return s_radar_ok; }