/* ============================================================
   RADAR MANAGER — DECLARAÇÃO v2.1
   ------------------------------------------------------------
   @file      radar_manager.h
   @version   2.1
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Driver para o sensor HLK-LD2450 via UART2.
   Versão 2.1 — correcção de conflito de tipos:
     radar_obj_t é definida AQUI e apenas aqui.
     O display_manager.h inclui este ficheiro para usar o tipo.

   REGRA ARQUITECTURAL:
   ---------------------
   radar_obj_t pertence ao módulo radar_manager porque:
     1. Representa dados produzidos pelo sensor físico
     2. O display_manager é consumidor, não produtor
     3. Evita definição duplicada e conflito de tipos

   Alterações v2.0 → v2.1:
   ------------------------
   1. CORRECÇÃO: radar_obj_t movida para cá (era também em
      display_manager.h causando "conflicting types")
   2. CORRECÇÃO: radar_manager_get_objects() declarada
      APENAS aqui (era também em display_manager.h)
   3. RADAR_TRAIL_MAX unificado (era RADAR_TRAIL_MAX_OBJ
      aqui e RADAR_TRAIL_MAX em display_manager.h)
   4. Constante RADAR_MAX_OBJ adicionada para consistência
      com display_manager.h anterior

   Dependências:
   -------------
   - hw_config.h    : RADAR_UART_PORT, RADAR_PIN_TX/RX, RADAR_BAUD_RATE
   - system_config.h: USE_RADAR, RADAR_DETECT_M
============================================================ */

#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================
   CONSTANTES DO RADAR
============================================================ */

/** Número máximo de alvos simultâneos (HLK-LD2450 suporta 3) */
#define MAX_RADAR_TARGETS   3

/**
 * Alias público — mesmo valor, nome consistente com o que o
 * display_manager usava anteriormente em display_manager.h.
 * Ambos os módulos devem usar RADAR_MAX_OBJ.
 */
#define RADAR_MAX_OBJ       MAX_RADAR_TARGETS

/** Comprimento máximo do rasto de cada objecto (frames) */
#define RADAR_TRAIL_MAX     8

/* ============================================================
   ESTRUTURA DE OBJECTO DO RADAR PARA O DISPLAY
   ------------------------------------------------------------
   FONTE ÚNICA DE VERDADE — definida apenas neste ficheiro.

   Coordenadas no sistema do sensor HLK-LD2450:
     x_mm : posição lateral em mm
              (negativo = esquerda do sensor,
               positivo = direita do sensor)
     y_mm : distância frontal em mm
              (0 = sensor, RADAR_MAX_MM = limite do alcance)
     trail_x / trail_y : histórico de posições (rasto visual)
     trail_len          : número de entradas válidas no rasto
============================================================ */
typedef struct {
    int     x_mm;                  /* posição lateral (mm)     */
    int     y_mm;                  /* distância frontal (mm)   */
    int     trail_x[RADAR_TRAIL_MAX]; /* rasto X (mm)          */
    int     trail_y[RADAR_TRAIL_MAX]; /* rasto Y (mm)          */
    uint8_t trail_len;             /* entradas válidas no rasto*/
} radar_obj_t;

/* ============================================================
   ESTRUTURA DE VEÍCULO DETECTADO (uso interno da FSM)
============================================================ */
typedef struct {
    bool    detected;   /* true se alvo activo              */
    float   distance;   /* distância Y em metros            */
    float   speed;      /* velocidade em km/h (valor abs.)  */
    int     x_mm;       /* posição lateral em mm (signed)   */
    int     y_mm;       /* distância frontal em mm          */
} radar_vehicle_t;

/** Conjunto de detecções do ciclo actual */
typedef struct {
    int             count;
    radar_vehicle_t targets[MAX_RADAR_TARGETS];
} radar_data_t;

/** Estrutura idêntica para injecção de dados simulados */
typedef radar_data_t radar_simulated_input_t;

/** Modos de operação do radar */
typedef enum {
    RADAR_MODE_DEFAULT   = 0,
    RADAR_MODE_UART,
    RADAR_MODE_SIMULATED,
    RADAR_MODE_NETWORK    /* reservado para uso futuro */
} radar_mode_t;

/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Inicializa radar no modo indicado.
 *        USE_RADAR=1 → configura UART2.
 *        USE_RADAR=0 → força RADAR_MODE_SIMULATED.
 * @param mode Modo de operação desejado
 */
void radar_init(radar_mode_t mode);

/**
 * @brief Lê frame UART e extrai dados dos alvos.
 *        Acumula bytes em buffer interno até ter frame
 *        completo de 30 bytes (protocolo HLK-LD2450).
 * @param out_data  Destino obrigatório (não pode ser NULL)
 * @param sim_input Dados simulados (só RADAR_MODE_SIMULATED;
 *                  ignorado em modo UART; pode ser NULL)
 * @return true se pelo menos um alvo activo detectado
 */
bool radar_read_data(radar_data_t            *out_data,
                     radar_simulated_input_t *sim_input);

/**
 * @brief Lê frame UART e preenche array radar_obj_t para
 *        o display_manager_set_radar().
 *        API principal para USE_RADAR=1 no main.c.
 *
 *        NOTA: Esta função é declarada APENAS aqui.
 *        A declaração anterior em display_manager.h foi
 *        removida na v2.1 para eliminar o conflito de tipos.
 *
 * @param objs  Array de destino (não pode ser NULL se max > 0)
 * @param max   Tamanho máximo do array (normalmente RADAR_MAX_OBJ)
 * @return Número de objectos activos (0..MAX_RADAR_TARGETS)
 */
uint8_t radar_manager_get_objects(radar_obj_t *objs, uint8_t max);

/**
 * @brief Retorna o modo de operação actual.
 * @return radar_mode_t corrente
 */
radar_mode_t radar_get_mode(void);

/**
 * @brief Verifica se algum alvo está dentro de RADAR_DETECT_M.
 * @param data Dados de detecção do ciclo actual
 * @return true se pelo menos um alvo dentro da zona activa
 */
bool radar_vehicle_in_range(const radar_data_t *data);

/**
 * @brief Velocidade do alvo mais próximo em km/h.
 * @param data Dados de detecção do ciclo actual
 * @return Velocidade em km/h ou 0 se nenhum alvo detectado
 */
float radar_get_closest_speed(const radar_data_t *data);

#endif /* RADAR_MANAGER_H */