/* ============================================================
   STATE MACHINE -- DECLARACAO
   ------------------------------------------------------------
   @file      state_machine.h
   @brief     Maquina de estados do Poste Inteligente
   @version   2.2
   @date      2026-03-15

   Alteracoes (v2.1 -> v2.2):
   --------------------------
   1. Adicionada state_machine_get_state_name() -- retorna o
      nome do estado actual como string para mostrar no display.
      Usado por display_manager_update_brightness() para
      mostrar "LUZ: 50%  [SAFE MODE]" no ecra.

   Dependencias:
   -------------
   - radar_manager.h (radar_vehicle_t)
   - comm_manager.h  (chamado internamente)
   - dali_manager.h  (chamado internamente)
   - system_config.h (constantes de tempo e brilho)
============================================================ */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include "radar_manager.h"

/* Estados do sistema */
typedef enum {
    STATE_IDLE      = 0,
    STATE_DETECTION,
    STATE_LIGHT_ON,
    STATE_TIMEOUT,
    STATE_SAFE_MODE
} system_state_t;

/* Inicializa FSM -- deve ser chamado apos comm_init() */
void state_machine_init(void);

/* Executa um ciclo da FSM -- chamado a cada 100ms pela fsm_task */
void state_machine_update(radar_vehicle_t radar_event, bool comm_ok);

/* Retorna estado actual */
system_state_t state_machine_get_state(void);

/* Retorna nome do estado actual para mostrar no display:
   "IDLE", "DETECTION", "LIGHT ON", "TIMEOUT", "SAFE MODE" */
const char *state_machine_get_state_name(void);

#endif /* STATE_MACHINE_H */