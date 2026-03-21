/* ============================================================
   RADAR MANAGER — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Interface com o sensor radar HLK-LD2450 via UART2.
   Detecta ate 3 alvos simultaneos com distancia e velocidade.
   USE_RADAR=0: modo simulado para desenvolvimento/teste.
   USE_RADAR=1: hardware UART real (producao).

   Ref HLK-LD2450: https://github.com/hermannsblum/ld2450
   Ref UART ESP-IDF: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html

   Dependencias:
   -------------
   - hw_config.h (RADAR_UART_PORT, RADAR_PIN_TX/RX, RADAR_BAUD_RATE)
   - system_config.h (USE_RADAR, RADAR_DETECT_M)
============================================================ */

#ifndef RADAR_MANAGER_H
#define RADAR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#define MAX_RADAR_TARGETS   3   /* HLK-LD2450 detecta ate 3 alvos */

/* Dados de um veiculo detectado */
typedef struct {
    bool  detected;     /* true se alvo valido            */
    float distance;     /* distancia em metros            */
    float speed;        /* velocidade em km/h             */
} radar_vehicle_t;

/* Conjunto de deteccoes do ciclo actual */
typedef struct {
    int            count;
    radar_vehicle_t targets[MAX_RADAR_TARGETS];
} radar_data_t;

/* Estrutura identica para injecao de dados simulados */
typedef radar_data_t radar_simulated_input_t;

/* Modos de operacao */
typedef enum {
    RADAR_MODE_DEFAULT   = 0,  /* UART real (alias de UART)  */
    RADAR_MODE_UART,           /* Hardware UART              */
    RADAR_MODE_SIMULATED,      /* Dados injectados           */
    RADAR_MODE_NETWORK         /* Reservado                  */
} radar_mode_t;

/* Inicializa radar no modo indicado */
void radar_init(radar_mode_t mode);

/* Le dados do radar.
   out_data : destino obrigatorio
   sim_input: dados simulados (so em RADAR_MODE_SIMULATED; pode ser NULL)
   Retorna true se pelo menos um alvo detectado */
bool radar_read_data(radar_data_t *out_data,
                     radar_simulated_input_t *sim_input);

/* Retorna modo de operacao actual */
radar_mode_t radar_get_mode(void);

/* Retorna true se algum alvo esta dentro de RADAR_DETECT_M */
bool radar_vehicle_in_range(const radar_data_t *data);

/* Retorna velocidade do alvo mais proximo (ou 0 se nenhum) */
float radar_get_closest_speed(const radar_data_t *data);

#endif /* RADAR_MANAGER_H */