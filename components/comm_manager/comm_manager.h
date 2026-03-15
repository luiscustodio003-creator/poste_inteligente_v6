/* ============================================================
   COMM MANAGER — DECLARAÇÃO
   ------------------------------------------------------------
   @file      comm_manager.h
   @brief     Gestão de comunicação entre postes inteligentes
   @version   3.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alterações (v2.0 → v3.0):
   --------------------------
   1. Adicionado comm_discover() — envia DISCOVER broadcast
   2. Adicionado comm_send_status() — propaga estado do poste
   3. Adicionado comm_get_neighbors() — expõe tabela de vizinhos
      ao display_manager para mostrar no ecrã

   Dependências:
   -------------
   - udp_manager.h (neighbor_t, neighbor_status_t)
   - wifi_manager.h
   - system_config.h
============================================================ */

#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include "udp_manager.h"    /* neighbor_t, neighbor_status_t, MAX_NEIGHBORS */

/* -----------------------------------------------------------
   comm_init — Inicializa UDP.
   ATENÇÃO: wifi_start_auto() deve ser chamado ANTES desta
   função em app_main(). comm_init() NÃO inicia o Wi-Fi.
   ----------------------------------------------------------- */
bool comm_init(void);

/* Envia DISCOVER broadcast para anunciar este poste */
void comm_discover(void);

/* Envia velocidade de veículo para vizinhos activos */
bool comm_send_vehicle(float speed);

/* Processa mensagem UDP recebida.
   Retorna true + preenche parâmetros se for SPD.
   Para DISCOVER/ACK/STATUS processa internamente. */
bool comm_receive_vehicle(float *speed, int *post_id, int *post_pos);

/* Propaga estado deste poste para todos os vizinhos directos */
void comm_send_status(neighbor_status_t status);

/* Retorna tabela de vizinhos para mostrar no display */
size_t comm_get_neighbors(neighbor_t *list, size_t max);

/* Verifica se a comunicação está operacional */
bool comm_status_ok(void);

#endif /* COMM_MANAGER_H */