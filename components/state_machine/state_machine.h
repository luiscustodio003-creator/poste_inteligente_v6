/* ============================================================
   STATE MACHINE — DECLARAÇÃO
   ------------------------------------------------------------
   @file      state_machine.h
   @brief     Máquina de estados do Poste Inteligente
   @version   2.8
   @date      2026-04-01

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Máquina de estados que controla a luminária e coordena
   a comunicação entre postes da cadeia.

   Estados:
   --------
     IDLE        T=0, Tc=0 — repouso, luz 10%
     LIGHT_ON    T>0 ou Tc>0 — luz 100%
     SAFE_MODE   Radar falhou E UDP falhou — luz 50% fixo
     MASTER      Líder da cadeia (viz. esq offline ou pos=0)
     AUTONOMO    Sem UDP — radar local assume comando
     OBSTACULO   Obstáculo estático persistente detectado (v2.8)
                 luz 100% contínuo, sinalização UDP a vizinhos

   Contadores:
   -----------
     T  = carros presentes (radar local confirmou)
     Tc = carros a caminho (UDP anunciou, ainda não chegaram)
     Luz 100% quando T>0 OU Tc>0 OU STATE_OBSTACULO
     Luz 10%  quando T=0 E Tc=0 (após TRAFIC_TIMEOUT_MS)

   Cenários de falha tratados (v2.8):
   -----------------------------------
   1. Radar falha mas UDP ok    → ignora radar, usa ETA por UDP
   2. Radar falha E UDP falha   → SAFE_MODE (50% fixo)
   3. Poste seguinte OFF        → Tc vai a zeros imediatamente
   4. Poste anterior OFF        → T limpo após T_STUCK_TIMEOUT_MS
   5. Sem vizinho esquerdo      → promove-se a MASTER
   6. pos=0 volta online        → MASTER_CLAIM em cadeia
   7. Radar recupera            → sai de SAFE_MODE/degradado para IDLE
   8. UDP recupera              → sai de AUTONOMO para estado anterior
   9. Obstáculo estático 8s+    → STATE_OBSTACULO, luz 100%, UDP OBST
  10. Obstáculo desaparece      → volta a IDLE

   Alterações v2.7 → v2.8:
   ------------------------
   1. STATE_OBSTACULO adicionado ao enum.
   2. SAFE_MODE corrigido: só activa quando radar E UDP falham.
      Antes: radar falha → SAFE_MODE (ignorava UDP).
      Agora: radar falha mas UDP ok → processa ETA normalmente.
   3. Transição SAFE_MODE → AUTONOMO quando UDP também falha.
   4. Recuperação do radar com Tc>0 — re-agenda ETA em vez de
      acender imediatamente (evita janela de luz sem carro).
   5. sm_is_obstaculo() getter adicionado.

   Dependências:
   -------------
   - dali_manager.h  : controlo PWM da luminária
   - comm_manager.h  : envio de mensagens UDP
   - radar_manager.h : radar_read_data(), radar_static_object_present()
   - system_config.h : TRAFIC_TIMEOUT_MS, LIGHT_*, POST_POSITION

============================================================ */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================
   ESTADOS
============================================================ */
typedef enum {
    STATE_IDLE       = 0, /* T=0 Tc=0 — repouso 10%               */
    STATE_LIGHT_ON,       /* T>0 ou Tc>0 — luz 100%               */
    STATE_SAFE_MODE,      /* Radar E UDP falhou — 50% fixo         */
    STATE_MASTER,         /* Líder da cadeia                       */
    STATE_AUTONOMO,       /* Sem UDP — radar local assume          */
    STATE_OBSTACULO       /* Obstáculo estático 8s+ — luz 100%     */
                          /* Sinalização de perigo para vizinhos   */
} system_state_t;

/* ============================================================
   INICIALIZAÇÃO E CICLO PRINCIPAL
============================================================ */

/**
 * @brief Inicializa FSM: T=0, Tc=0, IDLE, luz LIGHT_MIN.
 *        Deve ser chamada após dali_init() e comm_init().
 */
void state_machine_init(void);

/**
 * @brief Ciclo principal — chamar a 100ms pela fsm_task.
 *        Avalia timeouts, transições de estado e radar.
 * @param comm_ok  true se UDP operacional
 * @param is_master true se este poste é MASTER
 */
void state_machine_update(bool comm_ok, bool is_master);

/* ============================================================
   EVENTOS EXTERNOS
============================================================ */

/**
 * @brief Radar local detectou carro a velocidade vel km/h.
 *        → T++, Tc--, acende 100%, notifica vizinhos.
 */
void sm_on_radar_detect(float vel);

/**
 * @brief Notifica que o vizinho direito ficou OFFLINE.
 *        → Tc vai a zeros (não há quem receba as mensagens).
 */
void sm_on_right_neighbor_offline(void);

/**
 * @brief Notifica que o vizinho direito voltou online.
 *        → Retoma envio normal de TC_INC e SPD.
 */
void sm_on_right_neighbor_online(void);

/* ============================================================
   CALLBACKS UDP (implementações reais dos weak do udp_manager)
============================================================ */

/** Recebeu TC_INC com vel>0 — carro a caminho.
 *  Tc++, acende 100%, inicia timeout segurança,
 *  propaga TC_INC ao vizinho direito em cadeia (A→B→C→D...).
 */
void on_tc_inc_received(float speed);

/** Recebeu TC_INC com vel<0 — carro passou: T-- */
void on_prev_passed_received(void);

/** Recebeu SPD — guarda velocidade e refina timeout Tc com ETA real.
 *  Se eta_ms>0 e Tc>0, ajusta s_tc_timeout_ms = agora + eta_ms*2.
 */
void on_spd_received(float speed, uint32_t eta_ms);

/** Recebeu MASTER_CLAIM — cede liderança e propaga */
void on_master_claim_received(int from_id);

/* ============================================================
   TESTE SIMULADO (USE_RADAR=0)
============================================================ */

/**
 * @brief Injecta evento de carro simulado para teste.
 *        Equivalente a sm_on_radar_detect() mas sem hardware.
 *        Chamar do monitor série ou de um botão de teste.
 * @param vel Velocidade simulada em km/h (ex: 50.0)
 */
void sm_inject_test_car(float vel);

/* ============================================================
   GETTERS
============================================================ */

/** Estado actual da FSM */
system_state_t state_machine_get_state(void);

/** Nome do estado para display e logs */
const char *state_machine_get_state_name(void);

/** Contador T — carros presentes */
int state_machine_get_T(void);

/** Contador Tc — carros a caminho */
int state_machine_get_Tc(void);

/** Última velocidade detectada (km/h) */
float state_machine_get_last_speed(void);

/** true se radar local está operacional */
bool state_machine_radar_ok(void);

/** true se poste está em STATE_OBSTACULO */
bool sm_is_obstaculo(void);

/* ============================================================
   SIMULADOR — apenas USE_RADAR=0
============================================================ */

#if USE_RADAR == 0
/**
 * @brief Lê posição actual do carro simulado para o canvas.
 *        Chamada pelo main.c a cada ciclo de 100ms.
 * @param x_mm  Posição lateral em mm (pode ser NULL)
 * @param y_mm  Distância frontal em mm (pode ser NULL)
 * @return true se carro está visível no canvas
 */
bool sim_get_objeto(float *x_mm, float *y_mm);

/**
 * @brief Notifica o simulador que um carro vem a caminho.
 *        Chamada internamente por on_tc_inc_received().
 *        Activa o simulador local para mostrar o carro
 *        no canvas quando entrar no raio deste poste.
 * @param vel_kmh Velocidade do carro em km/h
 */
void sim_notificar_chegada(float vel_kmh);
#endif /* USE_RADAR == 0 */

#endif /* STATE_MACHINE_H */