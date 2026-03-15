/* ============================================================
   STATE MACHINE -- IMPLEMENTACAO
   ------------------------------------------------------------
   @file      state_machine.c
   @brief     Maquina de estados do Poste Inteligente
   @version   2.2
   @date      2026-03-15

   Alteracoes (v2.1 -> v2.2):
   --------------------------
   1. Adicionada state_machine_get_state_name() -- retorna
      o nome legivel do estado actual para o display_manager
      mostrar na linha "LUZ: XX%  [ESTADO]".

   Dependencias:
   -------------
   - state_machine.h
   - dali_manager.h
   - comm_manager.h
   - system_config.h
   - esp_timer, esp_log
============================================================ */

#include "state_machine.h"
#include "dali_manager.h"
#include "comm_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "system_config.h"

static const char *TAG = "FSM";

static system_state_t current_state    = STATE_IDLE;
static uint64_t       state_entered_ms = 0;

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static uint64_t time_in_state_ms(void)
{
    return now_ms() - state_entered_ms;
}

static const char *state_name(system_state_t s)
{
    switch (s)
    {
        case STATE_IDLE:      return "IDLE";
        case STATE_DETECTION: return "DETECTION";
        case STATE_LIGHT_ON:  return "LIGHT ON";
        case STATE_TIMEOUT:   return "TIMEOUT";
        case STATE_SAFE_MODE: return "SAFE MODE";
        default:              return "UNKNOWN";
    }
}

/* -----------------------------------------------------------
   transition_to -- Transicao com log e notificacao

   CORRECCAO v2.1: prev_state guardado ANTES de actualizar
   current_state para a condicao de notificacao ser correcta.
   ----------------------------------------------------------- */
static void transition_to(system_state_t new_state, const char *reason)
{
    system_state_t prev_state = current_state;  /* guarda ANTES */

    current_state    = new_state;
    state_entered_ms = now_ms();

    ESP_LOGI(TAG, "%s -> %s | %s",
             state_name(prev_state),
             state_name(new_state),
             reason);

    if (new_state == STATE_SAFE_MODE)
    {
        comm_send_status(NEIGHBOR_SAFE_MODE);
    }
    else if (new_state == STATE_IDLE &&
             prev_state == STATE_SAFE_MODE)
    {
        /* Saiu de SAFE_MODE -- comunicacao restabelecida */
        comm_send_status(NEIGHBOR_OK);
    }
    else if (new_state == STATE_IDLE)
    {
        /* Ciclo normal concluido */
        comm_send_status(NEIGHBOR_OK);
    }
}

/* ===========================================================
   INICIALIZACAO
   =========================================================== */

void state_machine_init(void)
{
    current_state    = STATE_IDLE;
    state_entered_ms = now_ms();
    dali_set_brightness(LIGHT_MIN);
    ESP_LOGI(TAG, "FSM inicializada | IDLE | Brilho=%d%%", LIGHT_MIN);
}

/* ===========================================================
   ACTUALIZACAO -- chamada pela fsm_task a cada 100ms
   =========================================================== */

void state_machine_update(radar_vehicle_t radar_event, bool comm_ok)
{
    float neighbor_speed = 0.0f;
    int   neighbor_id    = -1;
    int   neighbor_pos   = -1;

    bool spd_received = comm_receive_vehicle(&neighbor_speed,
                                             &neighbor_id,
                                             &neighbor_pos);
    if (spd_received)
    {
        ESP_LOGI(TAG, "Velocidade vizinho: %.1f km/h | ID=%d | pos=%d",
                 neighbor_speed, neighbor_id, neighbor_pos);
    }

    switch (current_state)
    {
        case STATE_IDLE:
            dali_set_brightness(LIGHT_MIN);
            if (!comm_ok)
                transition_to(STATE_SAFE_MODE, "comunicacao falhou");
            else if (radar_event.detected)
            {
                transition_to(STATE_DETECTION, "veiculo detectado");
                if (radar_event.speed > 0.0f)
                    comm_send_vehicle(radar_event.speed);
            }
            break;

        case STATE_DETECTION:
            dali_set_brightness((LIGHT_MAX + LIGHT_MIN) / 2);
            if (radar_event.detected)
                transition_to(STATE_LIGHT_ON, "veiculo confirmado");
            else if (time_in_state_ms() > DETECTION_TIMEOUT_MS)
                transition_to(STATE_IDLE, "falso alarme -- timeout");
            break;

        case STATE_LIGHT_ON:
            dali_set_brightness(LIGHT_MAX);
            if (radar_event.detected && radar_event.speed > 0.0f)
                comm_send_vehicle(radar_event.speed);
            if (!radar_event.detected)
                transition_to(STATE_TIMEOUT, "veiculo saiu");
            break;

        case STATE_TIMEOUT:
            dali_set_brightness(LIGHT_MIN);
            if (time_in_state_ms() >= LIGHT_ON_TIMEOUT_MS)
                transition_to(STATE_IDLE, "timeout concluido");
            else if (radar_event.detected)
                transition_to(STATE_LIGHT_ON, "veiculo voltou");
            break;

        case STATE_SAFE_MODE:
            if (radar_event.detected)
                dali_set_brightness(LIGHT_MAX);
            else
                dali_set_brightness(LIGHT_SAFE_MODE);
            if (comm_ok)
                transition_to(STATE_IDLE, "comunicacao restabelecida");
            break;

        default:
            ESP_LOGW(TAG, "Estado desconhecido -- reset IDLE");
            transition_to(STATE_IDLE, "reset");
            break;
    }
}

/* ===========================================================
   GETTERS
   =========================================================== */

system_state_t state_machine_get_state(void)
{
    return current_state;
}

/* Retorna nome legivel do estado para mostrar no display */
const char *state_machine_get_state_name(void)
{
    return state_name(current_state);
}