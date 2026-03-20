/* ============================================================
   UDP MANAGER — INTERFACE
   ------------------------------------------------------------
   @file      udp_manager.h
   @brief     Descoberta automática de vizinhos via UDP broadcast
   @version   2.1
   @date      2026-03-19

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere a comunicação UDP para descoberta automática de vizinhos
   na mesma rede. Envia broadcasts DISCOVER periodicamente e
   mantém uma tabela de vizinhos com timeout de expiração.
   Os vizinhos são classificados como esquerdo (ID < POSTE_ID)
   ou direito (ID > POSTE_ID) com base no POSTE_ID local.

   Protocolo:
   ----------
   Mensagem broadcast: "DISCOVER:<poste_id>"
   Porto: UDP_PORT (definido em system_config.h)
   Intervalo: DISCOVER_INTERVAL_MS
   Expiração: NEIGHBOR_TIMEOUT_MS sem recepção

   Dependências:
   -------------
   - system_config.h : UDP_PORT, MAX_NEIGHBORS, DISCOVER_INTERVAL_MS,
                       NEIGHBOR_TIMEOUT_MS, POSTE_ID
   - lwip            : sockets UDP
   - esp_timer       : timestamps de expiração
   - freertos        : task UDP

   
============================================================ */

#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

#include <stdint.h>

/* ============================================================
   Estrutura de vizinho descoberto
============================================================ */
typedef struct
{
    char     ip[16];        /* Endereço IP como string "x.x.x.x\0" */
    int      poste_id;      /* ID do poste vizinho                  */
    uint32_t last_seen;     /* Timestamp da última recepção (ms)    */
} neighbor_t;

/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Inicializa o socket UDP, regista a task de recepção
 *        e inicia o ciclo de broadcast DISCOVER.
 *        Deve ser chamada após wifi_manager_init() e após
 *        o Wi-Fi estar ligado (IP obtido).
 */
void udp_manager_init(void);

/**
 * @brief Processa um ciclo de gestão UDP (broadcast + timeout).
 *        Pode ser chamada periodicamente se não usar task interna.
 *        A task interna já chama esta lógica — uso opcional.
 * @param ms Milissegundos decorridos (não utilizado nesta versão)
 */
void udp_manager_tick(uint32_t ms);

/**
 * @brief Preenche as strings nebL e nebR com os IPs dos vizinhos.
 *        Se não houver vizinho conhecido, preenche com "---".
 * @param nebL Buffer de saída para IP do vizinho esquerdo (min 16 bytes)
 * @param nebR Buffer de saída para IP do vizinho direito  (min 16 bytes)
 */
void udp_manager_get_neighbors(char *nebL, char *nebR);

#endif /* UDP_MANAGER_H */