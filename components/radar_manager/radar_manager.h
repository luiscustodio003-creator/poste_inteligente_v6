/* ============================================================
   RADAR MANAGER — DECLARAÇÃO v2.4
   ------------------------------------------------------------
   @file      radar_manager.h
   @brief     Interface para o sensor HLK-LD2450 e Simulação.

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno

   Alterações v2.3 → v2.4:
   ------------------------
   ADIÇÃO — Detector de obstáculo estático persistente.
     Novo campo frames_estaticos em radar_vehicle_t.
     Incrementado a cada frame em que o alvo tem speed ≈ 0
     e dist_mm estável (variação < OBSTACULO_DIST_TOL_MM).
     Nova função radar_static_object_present() — devolve true
     quando pelo menos um alvo atingiu OBSTACULO_MIN_FRAMES
     consecutivos parado na mesma posição.
     Usado pela FSM para transitar para STATE_OBSTACULO.

   Parâmetros de detecção de obstáculo (configuráveis):
     OBSTACULO_SPEED_MAX_KMH  : velocidade máxima para ser "parado"
     OBSTACULO_DIST_TOL_MM    : variação máxima de dist permitida
     OBSTACULO_MIN_FRAMES     : mínimo de frames consecutivos
     OBSTACULO_MIN_FRAMES × 100ms = tempo mínimo de persistência
     Com OBSTACULO_MIN_FRAMES=80 → 8 segundos.

   Alterações v2.2 → v2.3 (mantidas):
   ------------------------------------
   CORRECÇÃO 1 — Campo speed_signed adicionado.
   CORRECÇÃO 2 — radar_vehicle_in_range() filtra por direcção.
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

/*
 * Limiar de velocidade para considerar objecto "a afastar-se".
 */
#define AFASTAR_THRESHOLD_KMH   2.0f

/*
 * Parâmetros do detector de obstáculo estático (v2.4)
 * ----------------------------------------------------
 * OBSTACULO_SPEED_MAX_KMH : velocidade máxima para considerar
 *   o alvo "parado". Abaixo deste valor + oscilações do sensor.
 *   Valor: 3.0 km/h — cobre micro-vibrações do veículo avariado.
 *
 * OBSTACULO_DIST_TOL_MM : variação máxima de distância permitida
 *   entre frames consecutivos para considerar "mesma posição".
 *   Valor: 300mm — margem para imprecisão do sensor em alvos lentos.
 *
 * OBSTACULO_MIN_FRAMES : número mínimo de frames consecutivos
 *   com alvo parado para confirmar obstáculo real.
 *   100ms/frame × 80 frames = 8 segundos de persistência mínima.
 *   Suficiente para descartar pássaros, folhas e rajadas de vento.
 *   Um carro em avaria permanece muito mais tempo.
 */
#define OBSTACULO_SPEED_MAX_KMH   3.0f
#define OBSTACULO_DIST_TOL_MM     300
#define OBSTACULO_MIN_FRAMES      80

/* --- Tipos de Dados --- */

typedef enum {
    RADAR_MODE_DEFAULT   = 0,
    RADAR_MODE_UART,
    RADAR_MODE_SIMULATED,
    RADAR_MODE_NETWORK
} radar_mode_t;

/** Estrutura para o Canvas do Display (com rasto e velocidade) */
typedef struct {
    int     x_mm;
    int     y_mm;
    int     trail_x[RADAR_TRAIL_MAX];
    int     trail_y[RADAR_TRAIL_MAX];
    uint8_t trail_len;
    float   speed_kmh;   /* velocidade deste alvo em km/h (módulo, para display) */
} radar_obj_t;

/**
 * Dados detalhados de detecção para a FSM.
 *
 * speed        : módulo da velocidade em km/h (sempre >= 0)
 * speed_signed : velocidade com sinal em km/h (v2.3)
 *                Negativo = a aproximar-se | Positivo = a afastar-se
 * frames_estaticos : contador de frames consecutivos com alvo parado
 *                na mesma posição (v2.4). Incrementado pelo radar_manager
 *                quando speed < OBSTACULO_SPEED_MAX_KMH e variação de
 *                distância < OBSTACULO_DIST_TOL_MM. Usado pela FSM
 *                para detectar obstáculos persistentes.
 */
typedef struct {
    bool    detected;
    float   distance;          /* Distância em metros                     */
    float   speed;             /* Velocidade em km/h (módulo, >= 0)       */
    float   speed_signed;      /* Velocidade em km/h com sinal (v2.3)     */
    int     x_mm;
    int     y_mm;
    uint16_t frames_estaticos; /* Frames consecutivos parado (v2.4)       */
    int      dist_mm_anterior; /* Distância no frame anterior (v2.4)      */
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

/** @brief Lê frame da UART, valida e extrai alvos com sinal de velocidade. */
bool radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input);

/** @brief API para o display_manager obter objectos prontos a desenhar. */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max);

/** @brief Verifica se o sensor está a responder (Heartbeat de hardware). */
bool radar_is_connected(void);

/** @brief Limpa backlog UART acumulado durante o arranque. */
void radar_flush_rx(void);

/**
 * @brief Detecta automaticamente o baud rate do HLK-LD2450.
 * @return Baud rate detectado, ou 0 se nenhum funcionar.
 */
int radar_auto_detect_baud(void);

/**
 * @brief Retorna string de estado: "REAL", "SIM" ou "FAIL".
 */
const char *radar_get_status_str(void);

/**
 * @brief Janela de diagnóstico interactiva (8 segundos).
 *        Imprime coordenadas, distância, velocidade e direcção
 *        de cada alvo. Chamar após radar_auto_detect_baud().
 */
void radar_diagnostic(void);

/**
 * @brief Getters utilitários para a FSM.
 *
 * radar_vehicle_in_range():
 *   Devolve true se há pelo menos um alvo a APROXIMAR-SE
 *   (speed_signed <= AFASTAR_THRESHOLD_KMH).
 *   Objectos claramente a afastar-se são ignorados.
 *
 * radar_get_closest_speed():
 *   Devolve o módulo de velocidade do alvo mais próximo
 *   que se está a aproximar (speed_signed <= threshold).
 *   Retorna 0.0 se todos os alvos se estão a afastar.
 */
radar_mode_t radar_get_mode(void);
bool         radar_vehicle_in_range(const radar_data_t *data);
float        radar_get_closest_speed(const radar_data_t *data);

/**
 * @brief Verifica se há obstáculo estático persistente no campo.
 *        Devolve true se pelo menos um alvo atingiu
 *        OBSTACULO_MIN_FRAMES consecutivos com velocidade
 *        < OBSTACULO_SPEED_MAX_KMH e posição estável.
 *        Usado pela FSM para transitar para STATE_OBSTACULO.
 *        Disponível apenas em RADAR_MODE_UART (hardware real).
 * @param data  Dados do último frame lido por radar_read_data().
 * @return true se obstáculo confirmado, false caso contrário.
 */
bool radar_static_object_present(const radar_data_t *data);

#endif /* RADAR_MANAGER_H */