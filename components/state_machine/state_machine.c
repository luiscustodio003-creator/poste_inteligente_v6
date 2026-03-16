/* ============================================================
   STATE MACHINE — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Implementacao completa da logica do simulador v6.
   Contadores T e Tc controlam o acendimento e apagamento.
   A luz nunca vai a 0% -- minimo e LIGHT_MIN (10%).

   Fluxo principal quando radar deteta carro:
   -------------------------------------------
   1. Tc-- (era "a caminho", agora chegou)
   2. T++  (esta aqui)
   3. Acende 100%
   4. Notifica vizinho anterior: T[prev]--
   5. Envia TC_INC para vizinho direito: Tc[next]++
   6. Calcula ETA e envia SPD para vizinho direito
   7. Agenda T-- em (RADAR_DETECT_M/vel + TRAFIC_TIMEOUT_MS)

   Ref simulador: simulador_v6.html
============================================================ */

#include "state_machine.h"
#include "dali_manager.h"
#include "comm_manager.h"
#include "system_config.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static const char *TAG = "FSM";

/* -----------------------------------------------------------
   Estado interno
   ----------------------------------------------------------- */
static system_state_t s_state = STATE_IDLE;
static int            s_T     = 0;  /* Carros presentes (radar confirmou) */
static int            s_Tc    = 0;  /* Carros a caminho (UDP informou)    */

/* Timestamp do ultimo carro detectado -- para calcular timeout */
static uint64_t s_last_detect_ms = 0;

/* Flag de apagamento agendado -- evita apagar se chegou novo carro */
static bool s_apagar_pendente = false;

/* Ultima velocidade detectada */
static float s_last_speed = 0.0f;

/* ===========================================================
   UTILITARIOS
   =========================================================== */

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

/* Retorna nome do estado para logs e display */
const char *state_machine_get_state_name(void)
{
    switch (s_state)
    {
        case STATE_IDLE:      return "IDLE";
        case STATE_LIGHT_ON:  return "LIGHT ON";
        case STATE_SAFE_MODE: return "SAFE MODE";
        case STATE_MASTER:    return "MASTER";
        case STATE_AUTONOMOUS:return "AUTONOMO";
        default:              return "---";
    }
}

/* Aplica brilho conforme estado actual dos contadores */
static void aplicar_brilho(void)
{
    if (s_T > 0 || s_Tc > 0)
    {
        /* Ha carros presentes ou a caminho -- 100% */
        dali_set_brightness(LIGHT_MAX);
        s_state = STATE_LIGHT_ON;
    }
    else
    {
        /* Nenhum carro -- volta ao minimo (nunca apaga) */
        dali_set_brightness(LIGHT_MIN);
        s_state = STATE_IDLE;
        s_last_speed = 0.0f;
    }
}

/* Verifica se pode voltar a 10% e agenda se necessario */
static void verificar_apagar(void)
{
    if (s_T <= 0 && s_Tc <= 0)
    {
        /* Liga flag -- state_machine_update verificara em 5s */
        s_apagar_pendente = true;
        s_last_detect_ms  = now_ms();
        ESP_LOGD(TAG, "Apagar agendado em %lums", (unsigned long)TRAFIC_TIMEOUT_MS);
    }
    else
    {
        ESP_LOGD(TAG, "Mantem 100%% T=%d Tc=%d", s_T, s_Tc);
    }
}

/* ===========================================================
   INICIALIZACAO
   =========================================================== */

void state_machine_init(void)
{
    s_state          = STATE_IDLE;
    s_T              = 0;
    s_Tc             = 0;
    s_last_detect_ms = 0;
    s_apagar_pendente= false;
    s_last_speed     = 0.0f;

    dali_set_brightness(LIGHT_MIN);
    ESP_LOGI(TAG, "FSM iniciada | IDLE | %d%%", LIGHT_MIN);
}

/* ===========================================================
   EVENTO: RADAR DETETA CARRO
   -----------------------------------------------------------
   Chamado pela fsm_task quando g_radar_event.detected == true
   e g_radar_event.distance <= RADAR_DETECT_M.
   =========================================================== */

void sm_on_radar_detect(float vel)
{
    s_apagar_pendente = false; /* Cancela apagamento pendente */
    s_last_detect_ms  = now_ms();
    s_last_speed      = vel;

    /* Converte Tc->T: carro era "a caminho", agora chegou */
    if (s_Tc > 0) s_Tc--;
    s_T++;

    /* Acende imediatamente a 100% */
    dali_set_brightness(LIGHT_MAX);
    s_state = STATE_LIGHT_ON;

    ESP_LOGI(TAG, "Radar: %.1fkm/h | T=%d Tc=%d", vel, s_T, s_Tc);

    /* Notifica vizinho anterior -- carro ja passou por ele (T[prev]--) */
    comm_notify_prev_passed(vel);

    /* Informa vizinho direito -- carro a caminho (Tc[next]++) */
    comm_send_tc_inc(vel);

    /* Envia SPD com ETA calculado para vizinho direito acender no momento */
    comm_send_spd(vel);

    /* Agenda decremento de T apos carro sair da zona.
       Tempo = tempo do carro atravessar a zona do radar (RADAR_DETECT_M * 2)
       + TRAFIC_TIMEOUT_MS de espera */
    float speed_ms = vel > 0 ? vel / 3.6f : 5.0f;
    uint32_t zona_ms = (uint32_t)((RADAR_DETECT_M * 2.0f / speed_ms) * 1000.0f);

    ESP_LOGD(TAG, "Carro sai da zona em %lums", (unsigned long)(zona_ms + TRAFIC_TIMEOUT_MS));
}

/* ===========================================================
   EVENTO: RECEBEU TC_INC VIA UDP
   -----------------------------------------------------------
   Vizinho anterior detectou carro a caminho.
   Incrementa Tc e acende 100% antecipadamente.
   =========================================================== */

void sm_on_tc_inc(float vel)
{
    s_apagar_pendente = false;
    s_last_speed      = vel;
    s_Tc++;

    /* Acende imediatamente -- carro esta a caminho */
    dali_set_brightness(LIGHT_MAX);
    s_state = STATE_LIGHT_ON;

    ESP_LOGI(TAG, "TC_INC: %.1fkm/h | T=%d Tc=%d", vel, s_T, s_Tc);
}

/* ===========================================================
   EVENTO: RECEBEU SPD VIA UDP
   -----------------------------------------------------------
   Informativo -- TC_INC ja tratou o Tc++.
   Guarda a velocidade para o display.
   =========================================================== */

void sm_on_spd_received(float vel, uint32_t eta_ms)
{
    s_last_speed = vel;
    ESP_LOGD(TAG, "SPD recebido: %.1fkm/h ETA:%lums", vel, (unsigned long)eta_ms);
}

/* ===========================================================
   EVENTO: VIZINHO ANTERIOR NOTIFICOU QUE CARRO PASSOU
   -----------------------------------------------------------
   O radar do vizinho seguinte detectou o carro, portanto
   o carro ja saiu da zona deste poste. Decrementa T.
   =========================================================== */

void sm_on_prev_passed(void)
{
    if (s_T > 0) s_T--;
    ESP_LOGD(TAG, "Carro passou anterior | T=%d Tc=%d", s_T, s_Tc);
    verificar_apagar();
}

/* ===========================================================
   CICLO PRINCIPAL — Chamado a 100ms pela fsm_task
   -----------------------------------------------------------
   Verifica se e hora de voltar a 10% (T=0, Tc=0, timeout).
   Gere estados SAFE_MODE, MASTER e AUTONOMOUS.
   =========================================================== */

void state_machine_update(bool comm_ok, bool is_master)
{
    /* Verifica se ha apagamento pendente e se o timeout passou */
    if (s_apagar_pendente && s_T <= 0 && s_Tc <= 0)
    {
        uint64_t elapsed = now_ms() - s_last_detect_ms;
        if (elapsed >= TRAFIC_TIMEOUT_MS)
        {
            s_apagar_pendente = false;
            aplicar_brilho();   /* Volta a 10% */
            ESP_LOGI(TAG, "T=0 Tc=0 timeout -> 10%%");

            /* Notifica vizinhos que zona esta limpa */
            comm_send_status(NEIGHBOR_OK);
        }
    }

    /* Sem comunicacao -- modo autonomo (radar local assume) */
    if (!comm_ok && s_state != STATE_SAFE_MODE)
    {
        /* Apenas entra em autonomo se nao houver trafego activo */
        if (s_T == 0 && s_Tc == 0)
        {
            s_state = STATE_AUTONOMOUS;
            ESP_LOGW(TAG, "Sem comunicacao -> AUTONOMO");
        }
    }

    /* Recupera de autonomo quando comunicacao volta */
    if (comm_ok && s_state == STATE_AUTONOMOUS)
    {
        s_state = is_master ? STATE_MASTER : STATE_IDLE;
        aplicar_brilho();
        ESP_LOGI(TAG, "Comunicacao restabelecida -> %s",
                 state_machine_get_state_name());

        /* Se for pos=0, reclama lideranca */
        if (POST_POSITION == 0)
            comm_send_master_claim();
    }

    /* Actualiza estado MASTER */
    if (is_master && s_state == STATE_IDLE)
        s_state = STATE_MASTER;
    else if (!is_master && s_state == STATE_MASTER)
        s_state = STATE_IDLE;
}

/* ===========================================================
   GETTERS
   =========================================================== */

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