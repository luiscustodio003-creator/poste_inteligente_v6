/* ============================================================
   RADAR MANAGER MODULE - HLK-LD2450
   ------------------------------------------------------------
   @file radar_manager.h
   @brief Interface para radar HLK-LD2450

   Projeto: Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)
   ============================================================ */

#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#define MAX_RADAR_TARGETS   8

/* ============================================================
   Veículo detetado
   ============================================================ */
typedef struct {
    bool  detected;
    float distance;   /**< metros  */
    float speed;      /**< km/h    */
} radar_vehicle_t;

/* ============================================================
   Conjunto de deteções do radar
   ============================================================ */
typedef struct {
    int            count;
    radar_vehicle_t targets[MAX_RADAR_TARGETS];
} radar_data_t;

/* radar_simulated_input_t é estruturalmente idêntico a radar_data_t.
 * Definido como typedef separado para clareza semântica na API. */
typedef radar_data_t radar_simulated_input_t;

/* ============================================================
   Modos de operação
   ============================================================ */
typedef enum {
    RADAR_MODE_DEFAULT   = 0,  /**< UART real (alias de UART)  */
    RADAR_MODE_UART,           /**< UART hardware              */
    RADAR_MODE_SIMULATED,      /**< Dados injetados por software */
    RADAR_MODE_NETWORK         /**< Reservado para uso futuro  */
} radar_mode_t;

/* ============================================================
   API pública
   ============================================================ */

/** Inicializa o radar no modo indicado.
 *  Se USE_RADAR==0 em system_config.h, o modo UART é ignorado. */
void radar_init(radar_mode_t mode);

/** Lê dados do radar.
 *  @param out_data   destino obrigatório
 *  @param sim_input  dados simulados (apenas em RADAR_MODE_SIMULATED; pode ser NULL)
 *  @return true se pelo menos um alvo detetado */
bool radar_read_data(radar_data_t *out_data, radar_simulated_input_t *sim_input);

/** Retorna o modo de operação atual. */
radar_mode_t radar_get_mode(void);

#endif /* RADAR_MANAGER_H */
