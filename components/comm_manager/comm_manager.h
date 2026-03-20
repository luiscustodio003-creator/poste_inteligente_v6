/* ============================================================
   COMM MANAGER — DECLARAÇÃO
   ------------------------------------------------------------
   @file      comm_manager.h
   @brief     Camada de abstracção sobre o udp_manager
   @version   2.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Camada de alto nível sobre o udp_manager. Calcula o ETA
   automaticamente com base em POSTE_DIST_M e RADAR_DETECT_M,
   gere a lógica de liderança MASTER e fornece acesso aos
   vizinhos por posição relativa (esquerdo / direito).

   Lógica de vizinhos:
   --------------------
   Vizinho esquerdo → posição POST_POSITION - 1
   Vizinho direito  → posição POST_POSITION + 1
   POST_POSITION=0 não tem vizinho esquerdo (é sempre MASTER
   por defeito até um vizinho esquerdo aparecer online).

   Lógica MASTER:
   ---------------
   Este poste é MASTER quando:
     - POST_POSITION == 0, OU
     - Vizinho esquerdo está OFFLINE ou inexistente

   Quando poste com POST_POSITION=0 volta online após falha,
   envia MASTER_CLAIM em cadeia para a direita.

   Dependências:
   -------------
   - udp_manager.h  : neighbor_t, neighbor_status_t, funções UDP
   - wifi_manager.h : wifi_manager_is_connected()
   - system_config.h: POST_POSITION, POSTE_DIST_M, RADAR_DETECT_M

   Alterações v1.0 → v2.0:
   ------------------------
   1. comm_init() corrigido: usa wifi_manager_is_connected()
      em vez de wifi_is_connected() / wifi_is_ap_active()
   2. Todas as chamadas alinhadas com udp_manager v3.0
   3. neighbor_t.id em vez de .poste_id
   4. Verificação de retorno de udp_manager_init()

============================================================ */

#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include "udp_manager.h"

/* ============================================================
   INICIALIZAÇÃO
============================================================ */

/**
 * @brief Inicializa o UDP. O Wi-Fi deve estar activo antes.
 * @return true se inicializado com sucesso
 */
bool comm_init(void);

/* ============================================================
   DESCOBERTA
============================================================ */

/**
 * @brief Envia DISCOVER broadcast manualmente.
 */
void comm_discover(void);

/* ============================================================
   ENVIO DE EVENTOS DE TRÁFEGO
============================================================ */

/**
 * @brief Envia SPD com ETA calculado para o vizinho direito.
 *        ETA = (POSTE_DIST_M - RADAR_DETECT_M) / velocidade
 * @param speed Velocidade do carro (km/h)
 * @return true se enviado com sucesso
 */
bool comm_send_spd(float speed);

/**
 * @brief Envia TC_INC imediato para o vizinho direito.
 *        O vizinho incrementa Tc e acende a 100%.
 * @param speed Velocidade do carro (km/h)
 * @return true se enviado com sucesso
 */
bool comm_send_tc_inc(float speed);

/**
 * @brief Notifica vizinho esquerdo que o carro já passou.
 *        O vizinho decrementa T.
 * @param speed Velocidade do carro (km/h)
 * @return true se enviado com sucesso
 */
bool comm_notify_prev_passed(float speed);

/* ============================================================
   GESTÃO DE LINHA
============================================================ */

/**
 * @brief Propaga MASTER_CLAIM para o vizinho direito.
 *        Chamado pelo poste pos=0 ao voltar online.
 */
void comm_send_master_claim(void);

/**
 * @brief Envia STATUS para todos os vizinhos activos.
 * @param status Estado a propagar
 */
void comm_send_status(neighbor_status_t status);

/* ============================================================
   ACESSO A VIZINHOS
============================================================ */

/**
 * @brief Devolve ponteiro para vizinho esquerdo (pos-1).
 * @return neighbor_t* ou NULL se POST_POSITION==0 ou offline
 */
neighbor_t *comm_get_neighbor_left(void);

/**
 * @brief Devolve ponteiro para vizinho direito (pos+1).
 * @return neighbor_t* ou NULL se não encontrado
 */
neighbor_t *comm_get_neighbor_right(void);

/**
 * @brief Preenche lista com todos os vizinhos activos.
 * @param list Array de destino
 * @param max  Tamanho máximo do array
 * @return Número de entradas copiadas
 */
size_t comm_get_neighbors(neighbor_t *list, size_t max);

/* ============================================================
   ESTADO DA LINHA
============================================================ */

/**
 * @brief Verifica se este poste é o MASTER da linha.
 *        true se POST_POSITION==0 ou vizinho esq OFFLINE.
 */
bool comm_is_master(void);

/**
 * @brief Verifica se a comunicação UDP está operacional.
 */
bool comm_status_ok(void);

/**
 * @brief Verifica se o vizinho esquerdo está online.
 */
bool comm_left_online(void);

/**
 * @brief Verifica se o vizinho direito está online.
 */
bool comm_right_online(void);

#endif /* COMM_MANAGER_H */