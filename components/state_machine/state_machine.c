/* ============================================================
   STATE MACHINE - IMPLEMENTACAO v2.6
   ------------------------------------------------------------
   @file      state_machine.c
   @brief     Maquina de estados do Poste Inteligente
   @version   2.6
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   FLUXO CORRECTO T / Tc (v2.5 -> v2.6):
   ----------------------------------------
   Problema v2.5:
     on_tc_inc_received() propagava TC_INC em cadeia (A->B->C)
     mas NAO propagava o SPD com ETA - o poste C recebia TC_INC
     mas nunca recebia o ETA, ficando sem o timer de pre-acendimento.

     on_prev_passed_received() decrementava T localmente mas nao
     propagava a informacao em cadeia - em A->B->C, quando C detecta
     o carro e envia T-- para B, B decrementava T_B mas A nunca
     sabia que o carro tinha saido da zona de B.

   Solucao v2.6 - fluxo completo encadeado:

   ENVIO (poste A detecta carro):
     A: T_A++, Tc_A-- (se havia)
     A -> B: TC_INC(vel) + SPD(vel, ETA_B)   <- ETA calculado para B
     B: Tc_B++, agenda acendimento pre-ETA
     B -> C: TC_INC(vel) + SPD(vel, ETA_C)   <- ETA recalculado para C
     C: Tc_C++, agenda acendimento pre-ETA
     (propaga ate ao ultimo poste online)

   CONFIRMACAO (carro chega a B):
     B: Tc_B--, T_B++
     B -> A: notify_prev_passed (T_A--)       <- A sabe que carro saiu
     B -> C: TC_INC(vel) + SPD(vel, ETA_C)   <- C ja sabe mas confirma

   CONFIRMACAO (carro chega a C):
     C: Tc_C--, T_C++
     C -> B: notify_prev_passed (T_B--)       <- B sabe que carro saiu

   RESULTADO final (depois de carro passar por todos):
     T_A=0, Tc_A=0  -> luz A baixa apos timeout
     T_B=0, Tc_B=0  -> luz B baixa apos timeout
     T_C=0, Tc_C=0  -> luz C baixa apos timeout

   CORRECCAO em on_tc_inc_received():
     Quando B recebe TC_INC+SPD de A, propaga TC_INC+SPD para C.
     O ETA para C = ETA_A_para_B + (POSTE_DIST_M / vel_ms).
     Simplificacao: usa comm_send_tc_inc() + comm_send_spd() que
     recalculam o ETA com base na velocidade recebida.

   CORRECCAO em on_prev_passed_received():
     Quando B recebe T-- de C, B decrementa T_B E propaga T--
     para A (comm_notify_prev_passed) se T_B ainda > 0 apos
     o decremento - garante que A tambem decrementa.
     NOTA: A propagacao so ocorre se havia carro em transito
     confirmado (s_T > 0 antes do decremento).

   Dependencias:
   -------------
   - dali_manager.h  : controlo PWM da luminaria
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

/* Timeout de seguranca para T preso (viz. esq. offline) */
#ifndef T_STUCK_TIMEOUT_MS
#define T_STUCK_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 3)
#endif

/* Timeout de seguranca para Tc (carro nao chegou) */
#ifndef TC_TIMEOUT_MS
#define TC_TIMEOUT_MS  (TRAFIC_TIMEOUT_MS * 2)
#endif

/*
 * MARGEM DE ACENDIMENTO ANTECIPADO (ms)
 * -----------------------------------------
 * O poste B acende (MARGEM_ACENDER_MS) antes do ETA previsto.
 * Garante que a luz esta a 100% quando o carro chega.
 * Valor: 500ms - confortavel mesmo com variacao de velocidade.
 * Nao deve ser superior a 1/4 do ETA minimo esperado.
 */
#ifndef MARGEM_ACENDER_MS
#define MARGEM_ACENDER_MS   500UL
#endif

/* Leituras consecutivas para confirmar falha/recuperacao do radar */
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
static int            s_radar_fail_cnt = 0;
static int            s_radar_ok_cnt   = 0;
static bool           s_right_online   = true;

/* Timestamps de controlo (ms desde boot) */
static uint64_t s_last_detect_ms       = 0;  /* ultima deteccao local  */
static uint64_t s_left_offline_ms      = 0;  /* viz. esq. ficou offline */
static uint64_t s_tc_timeout_ms        = 0;  /* timeout seguranca Tc   */
static bool     s_left_was_offline     = false;

/*
 * NOVO v2.5 - Timer de acendimento antecipado
 */
static uint64_t s_acender_em_ms        = 0;

/*
 * NOVO v2.9 - Heartbeat MASTER_CLAIM periodico
 * -----------------------------------------------
 * O poste pos=0 envia MASTER_CLAIM em cadeia periodicamente
 * para garantir que toda a linha sabe quem e o lider.
 * Intervalo: 30 segundos - suficiente para detectar postes
 * que recuperaram sem precisar de um evento externo.
 */
#define MASTER_CLAIM_HEARTBEAT_MS   30000ULL
static uint64_t s_master_claim_ms      = 0;

/* ============================================================
   UTILITARIOS
============================================================ */
static uint64_t _agora_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* ============================================================
   SIMULADOR FISICO - USE_RADAR=0  v2.7
   ------------------------------------------------------------
   LOGICA CORRECTA DA CADEIA DE POSTES:

   Apenas o poste MASTER (POST_POSITION == 0) gera carros
   simulados. Os postes seguintes (IDLE) NAO geram carros -
   apenas recebem o carro via UDP (TC_INC + SPD) quando o
   poste anterior o detecta, e depois o seu proprio simulador
   "ve" o carro entrar no raio quando o UDP avisa.

   Fluxo fisico real:
   ------------------
   1. Poste A (MASTER, pos=0) gera carro a X km/h
   2. Carro move-se: y decresce de RADAR_MAX_MM ate 0
   3. Quando y <= RADAR_DETECT_MM: A detecta, envia TC_INC+SPD->B
   4. Quando y <= 0: carro saiu do raio de A
      -> A entra em SIM_SAIU (aguarda T=0 Tc=0)

   Em B (pos=1):
   5. B recebe TC_INC+SPD -> Tc_B++, agenda pre-acendimento
   6. O simulador de B recebe o carro via on_tc_inc_received()
      -> activa s_sim_carro no estado SIM_ENTRAR com a
        velocidade recebida e y=RADAR_MAX_MM
   7. Carro move-se em B, detectado quando y <= RADAR_DETECT_MM
   8. B detecta -> T_B++, Tc_B--, envia TC_INC+SPD->C, PASSED->A

   Em C (pos=2): mesmo processo de B.

   Resultado: o carro "viaja" fisicamente pela cadeia de postes.
   Cada poste ve o carro entrar e sair do seu raio.
   O canvas do display de cada poste mostra o carro a mover-se.

   Estados do simulador:
   ----------------------
   SIM_AGUARDA  : so MASTER - espera intervalo entre carros
   SIM_ENTRAR   : carro foi anunciado via UDP, entra no raio
   SIM_EM_VIA   : carro visivel, a mover-se
   SIM_DETECTADO: zona de deteccao atingida, evento disparado
   SIM_SAIU     : carro passou pelo sensor (y<=0)
============================================================ */
#if USE_RADAR == 0

typedef enum {
    SIM_AGUARDA   = 0,  /* so MASTER: aguarda intervalo          */
    SIM_ENTRAR,         /* carro anunciado via UDP, a entrar      */
    SIM_EM_VIA,         /* carro visivel, a mover-se             */
    SIM_DETECTADO,      /* zona de deteccao atingida             */
    SIM_SAIU,           /* carro passou, aguarda limpeza T/Tc    */
} sim_estado_t;

typedef struct {
    float        y_mm;          /* distancia ao sensor (mm)       */
    float        x_mm;          /* posicao lateral (mm)           */
    float        vy;            /* velocidade mm/ciclo (100ms)    */
    float        vel_kmh;       /* velocidade km/h                */
    sim_estado_t estado;
    uint64_t     t_inicio_ms;
    bool         injectado;     /* deteccao ja disparada          */
} sim_carro_t;

static sim_carro_t s_sim_carro = {0};

/* Intervalo entre carros (so MASTER): TRAFIC_TIMEOUT + 3s */
#define SIM_INTERVALO_MS  ((uint64_t)(TRAFIC_TIMEOUT_MS + 3000))
/* Distancia de deteccao em mm */
#define SIM_ZONA_MM       ((float)(RADAR_DETECT_M * 1000))

/* Velocidades rotativas dos carros simulados */
static const float s_sim_vels[] = { 30.0f, 50.0f, 80.0f, 50.0f };
static int         s_sim_vel_idx = 0;

/* ------------------------------------------------------------
   _sim_iniciar
   ------------------------------------------------------------
   Lanca novo carro. Chamado:
     - pelo MASTER automaticamente apos SIM_INTERVALO_MS
     - pelos postes IDLE quando recebem TC_INC via UDP
       (vel ja conhecida, x_mm pseudo-aleatorio)
------------------------------------------------------------ */
static void _sim_iniciar(float vel)
{
    /* X pseudo-aleatorio dentro de +/-384mm do centro */
    s_sim_carro.x_mm        = (float)((int32_t)
                               (_agora_ms() & 0xFF) - 128) * 3.0f;
    s_sim_carro.y_mm        = (float)RADAR_MAX_MM;
    s_sim_carro.vel_kmh     = vel;
    /* vy: mm por ciclo de 100ms
       vel km/h / 3.6 = m/s * 1000 = mm/s * 0.1 = mm/100ms */
    s_sim_carro.vy          = (vel / 3.6f) * 100.0f * 0.25f;
    s_sim_carro.estado      = SIM_EM_VIA;
    s_sim_carro.injectado   = false;
    s_sim_carro.t_inicio_ms = _agora_ms();

    ESP_LOGI("SIM", "Carro entrou | pos=%d | %.0f km/h | vy=%.0fmm/ciclo",
             POST_POSITION, vel, s_sim_carro.vy);
}

/* ------------------------------------------------------------
   sim_notificar_chegada
   ------------------------------------------------------------
   Chamado por on_tc_inc_received() quando um poste IDLE
   recebe aviso via UDP de que um carro vem a caminho.
   Activa o simulador local para mostrar o carro no canvas
   quando ele entrar no raio deste poste.
   Velocidade recebida via UDP para consistencia fisica.
------------------------------------------------------------ */
void sim_notificar_chegada(float vel_kmh)
{
    if (s_sim_carro.estado != SIM_AGUARDA &&
        s_sim_carro.estado != SIM_SAIU    &&
        s_sim_carro.estado != SIM_ENTRAR)
        return;

    float vy = (vel_kmh / 3.6f) * 100.0f * 0.25f; /* mm por ciclo 100ms */

    /*
     * O carro esta actualmente no poste anterior a percorrer
     * a distancia POSTE_DIST_M ate entrar no raio deste poste.
     * Distancia real ate ao limite do raio deste poste:
     *   dist_extra = POSTE_DIST_M - RADAR_MAX_M  (em mm)
     * O carro percorre essa distancia antes de aparecer no canvas.
     * Representamos isso com y_mm inicial maior que RADAR_MAX_MM:
     *   y_inicial = RADAR_MAX_MM + dist_extra_mm
     * Assim o carro so aparece no canvas quando y_mm decresce
     * abaixo de RADAR_MAX_MM - que coincide com o ETA calculado.
     */
    float dist_extra_mm = (float)((POSTE_DIST_M - RADAR_MAX_M) * 1000);
    if (dist_extra_mm < 0.0f) dist_extra_mm = 0.0f;

    s_sim_carro.vel_kmh     = vel_kmh;
    s_sim_carro.vy          = vy;
    s_sim_carro.y_mm        = (float)RADAR_MAX_MM + dist_extra_mm;
    s_sim_carro.x_mm        = (float)((int32_t)
                               (_agora_ms() & 0xFF) - 128) * 3.0f;
    s_sim_carro.estado      = SIM_ENTRAR;
    s_sim_carro.injectado   = false;
    s_sim_carro.t_inicio_ms = _agora_ms();

    ESP_LOGI("SIM", "Carro a caminho | pos=%d | %.0f km/h | dist=%.0fmm",
             POST_POSITION, vel_kmh,
             (float)RADAR_MAX_MM + dist_extra_mm);
}

/* ------------------------------------------------------------
   _sim_update
   ------------------------------------------------------------
   Avanca simulacao 1 passo (100ms).
   Chamado por state_machine_update() a cada ciclo da FSM.

   MASTER (pos=0): gera carros automaticamente.
   IDLE   (pos>0): carro entra via sim_notificar_chegada(),
                   movimenta-se e dispara deteccao local.
------------------------------------------------------------ */
static void _sim_update(void)
{
    uint64_t agora = _agora_ms();

    switch (s_sim_carro.estado)
    {
        /* ---- AGUARDA: so o MASTER actual gera carros ----
           Condicao: is_master (pode ser qualquer poste que
           ficou sem vizinho esquerdo, nao apenas pos=0).
           Isto cobre o cenario A->B(falha)->C onde C se torna
           MASTER do seu troco e deve gerar carros simulados. */
        case SIM_AGUARDA:
            if (s_state != STATE_MASTER) break; /* so MASTER gera */

            if (s_sim_carro.t_inicio_ms == 0)
                s_sim_carro.t_inicio_ms = agora;

            if ((agora - s_sim_carro.t_inicio_ms) >= SIM_INTERVALO_MS)
            {
                /* Selecciona velocidade rotativa */
                float vel = s_sim_vels[s_sim_vel_idx];
                s_sim_vel_idx = (s_sim_vel_idx + 1) %
                    (sizeof(s_sim_vels) / sizeof(s_sim_vels[0]));
                _sim_iniciar(vel);
            }
            break;

        /* ---- ENTRAR: carro a percorrer distancia entre postes ----
           y_mm comeca em RADAR_MAX_MM + dist_extra e decresce.
           O carro so aparece no canvas (SIM_EM_VIA) quando
           y_mm <= RADAR_MAX_MM - que corresponde fisicamente
           ao momento em que entra no alcance do radar deste poste.
           Timing: coincide com o ETA calculado pelo poste anterior. */
        case SIM_ENTRAR:
            s_sim_carro.y_mm -= s_sim_carro.vy;

            if (s_sim_carro.y_mm <= (float)RADAR_MAX_MM)
            {
                /* Limita ao maximo do raio - entra pela borda */
                s_sim_carro.y_mm   = (float)RADAR_MAX_MM;
                s_sim_carro.estado = SIM_EM_VIA;
                ESP_LOGI("SIM", "Carro entrou no raio | pos=%d | y=%.0fmm",
                         POST_POSITION, s_sim_carro.y_mm);
            }
            break;

        /* ---- EM_VIA / DETECTADO: move o carro no raio ---- */
        case SIM_EM_VIA:
        case SIM_DETECTADO:
            /* Avanca carro em direccao ao sensor */
            s_sim_carro.y_mm -= s_sim_carro.vy;

            /* Dispara deteccao UMA VEZ ao entrar na zona activa */
            if (!s_sim_carro.injectado &&
                s_sim_carro.y_mm <= SIM_ZONA_MM)
            {
                s_sim_carro.estado    = SIM_DETECTADO;
                s_sim_carro.injectado = true;
                ESP_LOGI("SIM", "Deteccao | pos=%d | %.0f km/h | y=%.0fmm",
                         POST_POSITION,
                         s_sim_carro.vel_kmh, s_sim_carro.y_mm);
                sm_on_radar_detect(s_sim_carro.vel_kmh);
            }

            /* Carro passou pelo sensor -> SIM_SAIU */
            if (s_sim_carro.y_mm <= 0.0f)
            {
                s_sim_carro.y_mm        = 0.0f;
                s_sim_carro.estado      = SIM_SAIU;
                s_sim_carro.t_inicio_ms = agora;
                ESP_LOGI("SIM", "Carro saiu do raio | pos=%d",
                         POST_POSITION);
            }
            break;

        /* ---- SAIU: aguarda T=0 Tc=0 + timeout para reiniciar ---- */
        case SIM_SAIU:
            if (s_T == 0 && s_Tc == 0 &&
                (agora - s_sim_carro.t_inicio_ms) >=
                (uint64_t)TRAFIC_TIMEOUT_MS)
            {
                s_sim_carro.estado      = SIM_AGUARDA;
                s_sim_carro.t_inicio_ms = agora;
                ESP_LOGI("SIM", "Pronto para proximo carro | pos=%d",
                         POST_POSITION);
            }
            break;
    }
}

/* ------------------------------------------------------------
   sim_get_objeto
   Exporta posicao do carro para o canvas do display.
   Retorna true apenas quando o carro esta visivel no raio.
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
   T>0 ou Tc>0 -> 100% | T=0 e Tc=0 -> LIGHT_MIN
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
    s_acender_em_ms    = 0;
    s_master_claim_ms  = 0;

    dali_set_brightness(LIGHT_MIN);
    ESP_LOGI(TAG, "FSM v2.6 iniciada | IDLE | %d%%", LIGHT_MIN);
}

/* ============================================================
   sm_on_radar_detect
   ------------------------------------------------------------
   Chamado quando o radar local confirma carro na zona.

   Fluxo v2.6 (corrigido):
     1. Cancela timer de acendimento antecipado
     2. Tc-- (era "a caminho", agora chegou - confirma)
     3. T++ (esta aqui)
     4. Acende 100%
     5. Notifica poste ESQUERDO: T[prev]--
     6. NOVO: notifica poste DIREITO: Tc[next]--
        (cancela o Tc que foi incrementado pela propagacao
         em cadeia - evita que o poste seguinte conte este
         carro duas vezes: uma da propagacao e outra do TC_INC)
     7. Envia TC_INC + SPD ao poste direito
        (Tc[next]++ - resultado liquido: Tc permanece em 1)

   Exemplo A->B->C:
     Passo 1: A detecta -> TC_INC->B->C  (Tc_B=1, Tc_C=1)
     Passo 2: B detecta -> PASSED->A (T_A--)
                        -> PASSED->C (Tc_C-- -> 0) <- NOVO
                        -> TC_INC->C (Tc_C++ -> 1) <- correcto
     Resultado: Tc_C=1 em vez de Tc_C=2
============================================================ */
void sm_on_radar_detect(float vel)
{
    s_acender_em_ms  = 0;
    s_apagar_pend    = false;
    s_last_detect_ms = _agora_ms();
    s_last_speed     = vel;
    s_radar_ok_cnt   = RADAR_OK_COUNT;

    /* Converte Tc->T */
    s_Tc = 0;
    s_acender_em_ms = 0;
    if (s_T < MAX_RADAR_TARGETS) s_T++;

    dali_set_brightness(LIGHT_MAX);
    if (s_state == STATE_IDLE)
        s_state = STATE_LIGHT_ON;

    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "[CARRO DETECTADO] | Poste %s (pos=%d)",
             POSTE_NAME, POST_POSITION);
    ESP_LOGI(TAG, "   Velocidade : %.0f km/h", vel);
    ESP_LOGI(TAG, "   T=%d  Tc=%d  -> luz 100%%", s_T, s_Tc);

    /* Notifica poste ESQUERDO */
    comm_notify_prev_passed(vel);
    ESP_LOGI(TAG, "   <- PASSED enviado ao poste esquerdo (T--)");

    if (s_right_online)
    {
        comm_send_tc_inc(vel);
        comm_send_spd(vel);
        ESP_LOGI(TAG, "   -> TC_INC+SPD enviado ao poste direito");
        ESP_LOGI(TAG, "     (Tc do poste seguinte vai a 1)");
    }
    else
    {
        ESP_LOGI(TAG, "   [AVISO] Sem poste direito - ultimo da linha");
        ESP_LOGI(TAG, "     T reset via timeout em %dms",
                 TRAFIC_TIMEOUT_MS);
        s_apagar_pend    = true;
        s_last_detect_ms = _agora_ms();
    }
    ESP_LOGI(TAG, "----------------------------------------");
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
            ESP_LOGW(TAG, "Viz. dir. OFFLINE - Tc=%d -> 0", s_Tc);
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
   Recebeu TC_INC(vel) do poste anterior - carro a caminho.

   Fluxo v2.6:
     1. Tc++
     2. NAO acende imediatamente - aguarda SPD com ETA
     3. Propaga TC_INC + SPD para vizinho direito
        (o SPD recalcula o ETA para o poste seguinte)

   CORRECCAO v2.6:
     v2.5 propagava apenas TC_INC em cadeia mas nao o SPD.
     Sem SPD, o poste C nunca recebia o ETA e ficava sem
     o timer de pre-acendimento (s_acender_em_ms = 0).
     Agora propaga TC_INC + SPD para garantir que TODOS
     os postes da cadeia recebem o ETA correctamente.
------------------------------------------------------------ */
void on_tc_inc_received(float speed)
{
    s_apagar_pend    = false;
    s_last_speed     = speed;
    s_Tc++;
    s_last_detect_ms = _agora_ms();
    s_tc_timeout_ms  = _agora_ms() + TC_TIMEOUT_MS;

    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "[TC_INC RECEBIDO] | Poste %s (pos=%d)",
             POSTE_NAME, POST_POSITION);
    ESP_LOGI(TAG, "   Carro a %.0f km/h vem a caminho", speed);
    ESP_LOGI(TAG, "   T=%d  Tc=%d  -> aguarda carro", s_T, s_Tc);

#if USE_RADAR == 0
    sim_notificar_chegada(speed);
    ESP_LOGI(TAG, "   Simulador activado - carro a entrar no raio");
#endif

    if (s_right_online)
    {
        comm_send_tc_inc(speed);
        comm_send_spd(speed);
        ESP_LOGI(TAG, "   -> Propagado TC_INC+SPD ao poste direito");
    }
    ESP_LOGI(TAG, "----------------------------------------");
}

/* ------------------------------------------------------------
   on_prev_passed_received
   ------------------------------------------------------------
   Recebeu notificacao do poste seguinte - carro saiu: T--

   CORRECCAO v2.6:
     Quando o poste seguinte confirma a chegada do carro,
     este poste deve decrementar TANTO T como Tc:
       T--  : carro saiu da zona local
       Tc-- : cancela Tc da propagacao anterior
     E propagar em cadeia para o poste esquerdo.
------------------------------------------------------------ */
void on_prev_passed_received(void)
{
    bool tinha_T  = (s_T  > 0);
    bool tinha_Tc = (s_Tc > 0);

    if (s_T  > 0) s_T--;
    s_Tc = 0;
    s_acender_em_ms = 0;

    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, "[PASSED RECEBIDO] | Poste %s (pos=%d)",
             POSTE_NAME, POST_POSITION);
    ESP_LOGI(TAG, "   Carro confirmado no poste seguinte");
    ESP_LOGI(TAG, "   T=%d  Tc=%d", s_T, s_Tc);

    if (s_T == 0 && s_Tc == 0)
        ESP_LOGI(TAG, "   Zona livre - aguarda %dms p/ apagar",
                 TRAFIC_TIMEOUT_MS);

    if (tinha_T || tinha_Tc)
    {
        comm_notify_prev_passed(s_last_speed);
        ESP_LOGI(TAG, "   <- PASSED propagado ao poste esquerdo");
    }
    ESP_LOGI(TAG, "----------------------------------------");

    _agendar_apagar();
}

/* ------------------------------------------------------------
   on_spd_received
   ------------------------------------------------------------
   Recebeu SPD(vel, eta_ms) - agora calcula quando acender.

   LOGICA CENTRAL DO PRE-ACENDIMENTO (v2.5):
     acender_em = agora + eta_ms - MARGEM_ACENDER_MS

   Se eta_ms <= MARGEM_ACENDER_MS o carro ja esta perto:
     acende imediatamente (nao vale a pena agendar).

   O ETA vem calculado pelo comm_manager do poste anterior:
     ETA = (POSTE_DIST_M - RADAR_DETECT_M) / (vel/3.6) * 1000
   Exemplo: 50km/h, 50m postes, 7m radar
     ETA = (50-7) / (50/3.6) * 1000 = 43/13.9 * 1000 ~ 3094ms
     acender_em = agora + 3094 - 500 = agora + 2594ms
     -> B acende 500ms antes de o carro chegar.
------------------------------------------------------------ */
void on_spd_received(float speed, uint32_t eta_ms)
{
    s_last_speed = speed;

    if (s_Tc <= 0)
    {
        /* Tc ja foi limpo (timeout ou carro ja passou) - ignora */
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
         * Timeout de seguranca Tc: se o carro nao chegar
         * em 2*ETA, limpa Tc (UDP seguinte pode ter perdido).
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
         * ETA muito curto - carro quase a chegar:
         * acende imediatamente sem agendar timer.
         */
        s_acender_em_ms = 0;
        dali_set_brightness(LIGHT_MAX);
        if (s_state != STATE_MASTER)
            s_state = STATE_LIGHT_ON;

        ESP_LOGI(TAG, "SPD: ETA=%lums < margem - acender IMEDIATO",
                 (unsigned long)eta_ms);
    }
}

/* ------------------------------------------------------------
   on_master_claim_received
   Recebeu MASTER_CLAIM - cede lideranca e propaga em cadeia.
------------------------------------------------------------ */
void on_master_claim_received(int from_id)
{
    ESP_LOGI(TAG, "MASTER_CLAIM de ID=%d - cedemos lideranca",
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
   Monitoriza saude do radar (USE_RADAR=1).
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
            ESP_LOGI(TAG, "Radar recuperado - saindo de SAFE_MODE");

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
            ESP_LOGW(TAG, "Radar sem resposta - SAFE_MODE");
        }
    }
}

/* ============================================================
   _verificar_vizinho_esquerdo
   ------------------------------------------------------------
   Gestao de MASTER e limpeza de T preso.
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
            ESP_LOGW(TAG, "Viz. esq. OFFLINE - aguardar T_STUCK");
        }

        if (s_T > 0)
        {
            uint64_t delta = _agora_ms() - s_left_offline_ms;
            if (delta >= T_STUCK_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "T preso (%d) - forcar T=0", s_T);
                s_T = 0;
                _agendar_apagar();
            }
        }
    }
    else if (left_online && s_left_was_offline)
    {
        s_left_was_offline = false;
        ESP_LOGI(TAG, "Viz. esq. voltou online");

        /*
         * Quando o vizinho esquerdo volta online (ex: B recuperou),
         * este poste pode estar em MASTER (foi promovido durante
         * a falha). O pos=0 (A) vai enviar MASTER_CLAIM em cadeia
         * assim que detectar B - mas B pode nao saber que deve
         * ficar em IDLE se ninguem lhe disser.
         *
         * Solucao: se este poste e MASTER e o vizinho esquerdo
         * voltou, envia MASTER_CLAIM para o vizinho direito
         * em cadeia, propagando a lideranca correctamente.
         * O pos=0 (A) nao precisa disto - e sempre MASTER.
         */
        if (s_state == STATE_MASTER && POST_POSITION > 0)
        {
            ESP_LOGI(TAG, "Viz. esq. voltou - cedendo MASTER em cadeia");
            s_state = STATE_IDLE;
            comm_send_master_claim();
        }
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
        ESP_LOGI(TAG, "MASTER cedido - volta a IDLE");
    }
}

/* ============================================================
   state_machine_update
   ------------------------------------------------------------
   Ciclo principal - chamar a 100ms pela fsm_task.

   Ordem de avaliacao:
     1. Le radar (hardware ou simulado)
     2. Verifica saude do radar
     3. SAFE_MODE se radar falhou
     4. Verifica vizinho direito (Tc=0 se offline)
     5. Verifica vizinho esquerdo (MASTER, T preso)
     6. NOVO v2.5: verifica timer de acendimento antecipado
     7. Timeout de apagamento (T=0,Tc=0 + TRAFIC_TIMEOUT_MS)
     8. Timeout Tc (carro nao chegou)
     9. Fallback seguranca: TC_INC sem SPD por >1s -> acende
    10. AUTONOMO <-> normal
============================================================ */
void state_machine_update(bool comm_ok, bool is_master)
{
    /* ----------------------------------------------------------
       1+2. Le radar e verifica saude
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
    teve_detecao = false; /* simulado - saude sempre OK */
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
            ESP_LOGW(TAG, "SAFE_MODE - luz %d%%", LIGHT_SAFE_MODE);
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
       6. NOVO v2.5 - Timer de acendimento antecipado
       ----------------------------------------------------------
       Verifica se chegou a hora de acender antes do carro.
       Condicao: s_acender_em_ms > 0 e agora >= s_acender_em_ms.
       So actua se Tc > 0 (ha carro previsto a caminho).
    ---------------------------------------------------------- */
    if (s_acender_em_ms > 0 && _agora_ms() >= s_acender_em_ms)
    {
        s_acender_em_ms = 0;  /* limpa timer - nao dispara de novo */

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
       7. Timeout de apagamento (T=0, Tc=0 -> luz 10%)
    ---------------------------------------------------------- */
    if (s_apagar_pend && s_T <= 0 && s_Tc <= 0)
    {
        uint64_t elapsed = _agora_ms() - s_last_detect_ms;
        if (elapsed >= TRAFIC_TIMEOUT_MS)
        {
            s_apagar_pend = false;
            _aplicar_brilho();
            ESP_LOGI(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "[IDLE] Poste %s -> IDLE | T=0 Tc=0",
                     POSTE_NAME);
            ESP_LOGI(TAG, "   Zona livre ha %lums -> luz %d%%",
                     (unsigned long)elapsed, LIGHT_MIN);
            ESP_LOGI(TAG, "----------------------------------------");

            if (comm_ok)
                comm_send_status(NEIGHBOR_OK);
        }
    }

    /* ----------------------------------------------------------
       8. Timeout Tc - carro nao chegou (UDP perdido)
    ---------------------------------------------------------- */
    if (s_Tc > 0 && s_tc_timeout_ms > 0 &&
        _agora_ms() >= s_tc_timeout_ms)
    {
        ESP_LOGW(TAG, "Tc timeout - carro nao chegou | Tc=%d->%d",
                 s_Tc, s_Tc - 1);
        s_Tc--;
        s_acender_em_ms = 0;   /* cancela timer de acendimento */
        s_tc_timeout_ms = (s_Tc > 0)
                          ? (_agora_ms() + TC_TIMEOUT_MS) : 0;
        _agendar_apagar();
    }

    /* ----------------------------------------------------------
       9. Fallback de seguranca:
          TC_INC recebido ha >1000ms mas SPD ainda nao chegou.
          Acende por precaucao para nao deixar o carro no escuro.
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

            ESP_LOGW(TAG, "Fallback: TC_INC sem SPD >1s - acender");
        }
    }

    /* ----------------------------------------------------------
       10. AUTONOMO <-> normal
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
                ESP_LOGW(TAG, "Sem UDP -> AUTONOMO");
            }
        }
    }
    else
    {
        if (s_state == STATE_AUTONOMO)
        {
            s_state = is_master ? STATE_MASTER : STATE_IDLE;
            _aplicar_brilho();
            ESP_LOGI(TAG, "UDP recuperado -> %s",
                     state_machine_get_state_name());

            if (POST_POSITION == 0)
            {
                ESP_LOGI(TAG, "pos=0 volta online -> MASTER_CLAIM");
                comm_send_master_claim();
                s_master_claim_ms = _agora_ms();
            }
        }
    }

    /* ----------------------------------------------------------
       11. Heartbeat MASTER_CLAIM periodico (v2.9)
       ----------------------------------------------------------
       O poste pos=0 envia MASTER_CLAIM em cadeia a cada
       MASTER_CLAIM_HEARTBEAT_MS (30s). Garante que postes
       que recuperaram de falha recebem a confirmacao de quem
       e o lider da linha, mesmo que nao haja trafego activo.

       Exemplo: B falhou e recuperou. A continuou a funcionar
       normalmente. B arranca sem saber o seu papel. No maximo
       30s depois, A envia MASTER_CLAIM -> B recebe -> B fica IDLE
       -> B propaga para C -> C cede MASTER e volta a IDLE.
    ---------------------------------------------------------- */
    if (POST_POSITION == 0 && comm_ok &&
        s_state == STATE_MASTER)
    {
        uint64_t agora = _agora_ms();
        if (s_master_claim_ms == 0 ||
            (agora - s_master_claim_ms) >= MASTER_CLAIM_HEARTBEAT_MS)
        {
            ESP_LOGI(TAG, "Heartbeat MASTER_CLAIM (cada %llus)",
                     MASTER_CLAIM_HEARTBEAT_MS / 1000ULL);
            comm_send_master_claim();
            s_master_claim_ms = agora;
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