/* ============================================================
   RADAR MANAGER — DECLARAÇÃO v2.2
   ------------------------------------------------------------
   @file      radar_manager.h
   @brief     Interface para o sensor HLK-LD2450 e Simulação.
   
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   
   Este módulo gere a comunicação UART com o sensor de radar
   e fornece os dados processados para a FSM e para o Display.
   ============================================================ */

#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "system_config.h"

/* --- Constantes do Radar --- */
#define MAX_RADAR_TARGETS   3
#define RADAR_MAX_OBJ       MAX_RADAR_TARGETS
#define RADAR_TRAIL_MAX     8

/* --- Tipos de Dados --- */

typedef enum {
    RADAR_MODE_DEFAULT   = 0,
    RADAR_MODE_UART,
    RADAR_MODE_SIMULATED,
    RADAR_MODE_NETWORK
} radar_mode_t;

/** Estrutura para o Canvas do Display (com rasto) */
typedef struct {
    int     x_mm;              
    int     y_mm;              
    int     trail_x[RADAR_TRAIL_MAX]; 
    int     trail_y[RADAR_TRAIL_MAX]; 
    uint8_t trail_len;         
} radar_obj_t;

/** Dados detalhados de detecção para a FSM */
typedef struct {
    bool    detected;   
    float   distance;   /* Distância em metros */
    float   speed;      /* Velocidade em km/h */
    int     x_mm;       
    int     y_mm;       
} radar_vehicle_t;

typedef struct {
    int             count;
    radar_vehicle_t targets[MAX_RADAR_TARGETS];
} radar_data_t;

/** Input para simulação (quando USE_RADAR=0) */
typedef struct {
    bool    active;
    int16_t distance;
} radar_simulated_input_t;

/* --- API Pública --- */

/** @brief Inicializa hardware UART2 ou modo simulado. */
void radar_init(radar_mode_t mode);

/** @brief Lê frame da UART, valida 'round bounds' e extrai alvos. */
bool radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input);

/** @brief API para o display_manager obter objectos prontos a desenhar. */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max);

/** @brief Verifica se o sensor está a responder (Heartbeat de hardware). */
bool radar_is_connected(void);

/** @brief Limpa backlog UART acumulado durante o arranque.
 *  Chamar após o delay inicial da fsm_task. */
void radar_flush_rx(void);

/**
 * @brief Detecta automaticamente o baud rate do HLK-LD2450.
 *        Tenta 256000, 115200 e 57600 baud (300ms por tentativa).
 *        Se encontrar frames válidos, configura a UART para esse baud.
 *        Chamar APÓS radar_init(RADAR_MODE_UART) e ANTES de radar_flush_rx().
 * @return Baud rate detectado, ou 0 se nenhum funcionar (mantém 115200).
 */
int radar_auto_detect_baud(void);

/**
 * @brief Retorna string de estado para o display: "REAL", "SIM" ou "FAIL".
 *        "REAL" → modo UART e a receber frames.
 *        "SIM"  → modo simulado.
 *        "FAIL" → modo UART mas sem frames válidos.
 */
const char *radar_get_status_str(void);

/** @brief Getters utilitários para a FSM */
radar_mode_t radar_get_mode(void);
bool radar_vehicle_in_range(const radar_data_t *data);
float radar_get_closest_speed(const radar_data_t *data);

#endif /* RADAR_MANAGER_H */