/* ============================================================
   STATE MACHINE — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Maquina de estados com contadores T e Tc por poste.
   Implementa a logica completa do simulador:

   Contadores:
     T  = carros presentes (radar confirmou chegada)
     Tc = carros a caminho (UDP informou, ainda nao chegaram)
     Luz 100% quando T>0 OU Tc>0
     Luz 10% apenas quando T=0 E Tc=0 (apos TRAFIC_TIMEOUT_MS)

   Eventos externos processados:
     sm_on_radar_detect(vel) : radar deteta carro
       -> T++, Tc--, acende 100%
       -> notifica anterior (T[prev]--)
       -> envia TC_INC e SPD para proximo

     sm_on_tc_inc(vel) : recebeu TC_INC via UDP
       -> Tc++, acende 100%

     sm_on_spd_received(vel, eta) : recebeu SPD via UDP
       -> informativo (TC_INC ja tratou Tc++)

     sm_on_prev_passed(vel) : anterior notificou que carro passou
       -> T-- no proprio poste

   Estados da linha:
     STATE_IDLE      : T=0, Tc=0, luz 10%
     STATE_LIGHT_ON  : T>0 ou Tc>0, luz 100%
     STATE_SAFE_MODE : radar falhou, luz 50%
     STATE_MASTER    : assumiu lideranca da linha
     STATE_AUTONOMOUS: sem comunicacao, radar local assume

   Dependencias:
   -------------
   - dali_manager.h, comm_manager.h, radar_manager.h
   - system_config.h (TRAFIC_TIMEOUT_MS, LIGHT_MIN/MAX/SAFE_MODE)
   - esp_timer, esp_log
============================================================ */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

/* Estados da linha */
typedef enum {
    STATE_IDLE       = 0,  /* T=0 Tc=0 -- repouso 10%              */
    STATE_LIGHT_ON,        /* T>0 ou Tc>0 -- luz 100%              */
    STATE_SAFE_MODE,       /* Radar falhou -- 50% fixo              */
    STATE_MASTER,          /* Lider da linha -- controla cadeia     */
    STATE_AUTONOMOUS       /* Sem UDP -- radar local assume comando */
} system_state_t;

/* Inicializa FSM -- T=0, Tc=0, IDLE, luz 10% */
void state_machine_init(void);

/* Ciclo principal -- chamado a 100ms pela fsm_task */
void state_machine_update(bool comm_ok, bool is_master);

/* Radar local deteta carro a velocidade vel (km/h) */
void sm_on_radar_detect(float vel);

/* Recebeu TC_INC via UDP -- carro a caminho */
void sm_on_tc_inc(float vel);

/* Recebeu SPD via UDP -- informativo */
void sm_on_spd_received(float vel, uint32_t eta_ms);

/* Vizinho anterior notificou que carro passou -- T-- */
void sm_on_prev_passed(void);

/* Retorna estado actual */
system_state_t state_machine_get_state(void);

/* Retorna nome do estado para display */
const char *state_machine_get_state_name(void);

/* Retorna contador T (carros presentes) */
int state_machine_get_T(void);

/* Retorna contador Tc (carros a caminho) */
int state_machine_get_Tc(void);

#endif /* STATE_MACHINE_H */