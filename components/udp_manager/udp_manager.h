/* ============================================================
   UDP MANAGER — DECLARAÇÃO
   ------------------------------------------------------------
   @file      udp_manager.h
   @brief     Comunicação UDP entre postes inteligentes
   @version   4.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alterações (v3.0 → v4.0):
   --------------------------
   1. Adicionado neighbor_status_t (OK / OFFLINE / SAFE_MODE)
   2. neighbor_t agora inclui: last_speed, last_seen, status
   3. Adicionada udp_manager_process() — processa DISCOVER, ACK,
      SPD e STATUS numa única chamada (para a udp_rx_task)
   4. Adicionada udp_manager_send_ack() — envia confirmação
   5. Adicionada udp_manager_send_status() — propaga estado
   6. Adicionada udp_manager_load_neighbors_nvs() — carrega
      vizinhos persistidos na NVS ao arrancar
   7. Adicionada udp_manager_save_neighbors_nvs() — persiste
      tabela de vizinhos na NVS após cada alteração

   Protocolo de mensagens UDP (texto simples):
   -------------------------------------------
   DISCOVER:<id>:<name>:<pos>
     → Broadcast de anúncio. Receptor regista o emissor
       e responde com DISCOVER_ACK.

   DISCOVER_ACK:<id>:<name>:<pos>:<ip>
     → Resposta directa ao DISCOVER. Confirma registo.

   SPD:<id>:<name>:<pos>:<velocidade>
     → Dados de velocidade de veículo detectado.

   ACK:<id>:<msg_type>
     → Confirmação de recepção. msg_type = "SPD" ou "STATUS".

   STATUS:<id>:<pos>:<estado>
     → Propagação de estado. estado = "OK", "OFFLINE", "SAFE".

   Dependências:
   -------------
   - system_config.h (UDP_PORT, MAX_NEIGHBORS, MAX_IP_LEN,
                      NEIGHBOR_TIMEOUT_MS, ACK_TIMEOUT_MS)
   - wifi_manager.h  (wifi_get_ip)
   - nvs, lwip
============================================================ */

#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "system_config.h"

/* ===========================================================
   Estado do vizinho
   =========================================================== */
typedef enum
{
    NEIGHBOR_OK        = 0, /* Activo e a comunicar normalmente */
    NEIGHBOR_OFFLINE,       /* Sem resposta após NEIGHBOR_TIMEOUT_MS */
    NEIGHBOR_SAFE_MODE      /* Online mas em modo seguro autónomo   */
} neighbor_status_t;

/* ===========================================================
   Estrutura de vizinho
   =========================================================== */
typedef struct
{
    char               ip[MAX_IP_LEN];  /* IP do vizinho               */
    int                id;              /* ID do poste vizinho         */
    int                pos;             /* Posição na cadeia           */
    char               name[32];        /* Nome do poste               */
    uint64_t           last_seen;       /* Timestamp último contacto   */
    float              last_speed;      /* Última velocidade recebida  */
    neighbor_status_t  status;          /* Estado actual do vizinho    */
    bool               active;          /* Registo válido na tabela    */
} neighbor_t;

/* ===========================================================
   API — Inicialização
   =========================================================== */

/* Cria socket UDP, faz bind na UDP_PORT.
   Deve ser chamada após wifi_start_auto(). */
bool udp_manager_init(void);

/* Carrega tabela de vizinhos persistida na NVS.
   Chamada em udp_manager_init() automaticamente. */
void udp_manager_load_neighbors_nvs(void);

/* Persiste tabela de vizinhos na NVS. */
void udp_manager_save_neighbors_nvs(void);

/* ===========================================================
   API — Envio
   =========================================================== */

/* Envia mensagem para um IP específico */
bool udp_manager_send_to(const char *buffer, size_t len, const char *ip);

/* Envia DISCOVER em broadcast — anuncia este poste à rede */
void udp_manager_discover_neighbors(void);

/* Envia ACK de confirmação para um IP */
bool udp_manager_send_ack(const char *dest_ip, const char *msg_type);

/* Envia STATUS para vizinho directo */
bool udp_manager_send_status(const char *dest_ip,
                              neighbor_status_t status);

/* Constrói mensagem SPD no buffer */
void udp_build_spd(char *buffer, size_t max_len, float speed);

/* ===========================================================
   API — Recepção e processamento
   =========================================================== */

/* Processa uma mensagem recebida (não bloqueante — MSG_DONTWAIT).
   Trata: DISCOVER, DISCOVER_ACK, SPD, ACK, STATUS.
   Se for SPD, preenche speed_out e retorna true.
   Para todas as outras mensagens processa internamente. */
bool udp_manager_process(float  *speed_out,
                          int    *id_out,
                          int    *pos_out);

/* ===========================================================
   API — Tabela de vizinhos
   =========================================================== */

/* Copia vizinhos activos para list[]. Retorna contagem. */
size_t udp_manager_get_neighbors(neighbor_t *list, size_t max);

/* Retorna true se existir pelo menos um vizinho activo */
bool udp_manager_has_active_neighbors(void);

/* Marca vizinhos sem contacto há mais de NEIGHBOR_TIMEOUT_MS
   como NEIGHBOR_OFFLINE. Chamar periodicamente. */
void udp_manager_check_timeouts(void);

/* Retorna contagem total de vizinhos registados (activos + offline) */
size_t udp_manager_get_neighbor_count(void);

#endif /* UDP_MANAGER_H */