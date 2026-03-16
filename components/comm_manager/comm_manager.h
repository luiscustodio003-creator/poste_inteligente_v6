/* ============================================================
   COMM MANAGER — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Camada de abstraccao sobre o udp_manager.
   Calcula ETA automaticamente com base em POSTE_DIST_M e
   RADAR_DETECT_M de system_config.h.
   Gere lideranca MASTER_CLAIM e vizinhos por posicao.

   Logica de vizinhos:
   --------------------
   Vizinho esq = pos POST_POSITION - 1
   Vizinho dir = pos POST_POSITION + 1
   pos=0 nao tem vizinho esquerdo (e sempre master por defeito).

   Logica MASTER:
   ---------------
   Poste e MASTER quando vizinho esquerdo esta OFFLINE ou inexistente.
   pos=0 tem prioridade maxima: ao voltar, propaga MASTER_CLAIM.

   Dependencias:
   -------------
   - udp_manager.h, wifi_manager.h, system_config.h
============================================================ */

#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include "udp_manager.h"

/* Inicializa UDP -- Wi-Fi deve estar activo antes */
bool comm_init(void);

/* Envia DISCOVER broadcast */
void comm_discover(void);

/* Envia SPD com ETA calculado para vizinho direito */
bool comm_send_spd(float speed);

/* Envia TC_INC imediato para vizinho direito (carro a caminho) */
bool comm_send_tc_inc(float speed);

/* Notifica vizinho esquerdo que carro passou (ele decrementa T) */
bool comm_notify_prev_passed(float speed);

/* Propaga MASTER_CLAIM para vizinho direito */
void comm_send_master_claim(void);

/* Envia STATUS para todos os vizinhos */
void comm_send_status(neighbor_status_t status);

/* Vizinho esquerdo (pos-1) ou NULL */
neighbor_t *comm_get_neighbor_left(void);

/* Vizinho direito (pos+1) ou NULL */
neighbor_t *comm_get_neighbor_right(void);

/* Tabela completa de vizinhos */
size_t comm_get_neighbors(neighbor_t *list, size_t max);

/* true se este poste e MASTER (vizinho esq offline ou inexistente) */
bool comm_is_master(void);

/* true se comunicacao operacional */
bool comm_status_ok(void);

/* true se vizinho esquerdo online */
bool comm_left_online(void);

/* true se vizinho direito online */
bool comm_right_online(void);

#endif /* COMM_MANAGER_H */