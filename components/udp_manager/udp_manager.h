/* ============================================================
   UDP MANAGER — INTERFACE
   ------------------------------------------------------------
   @file      udp_manager.h
   @brief     Comunicação UDP entre postes — protocolo completo
   @version   3.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere toda a comunicação UDP entre postes da cadeia.
   Implementa descoberta automática de vizinhos (DISCOVER)
   e o protocolo completo de mensagens para coordenação
   de tráfego e gestão de falhas.

   Protocolo de mensagens (texto simples, separado por ':'):
   ----------------------------------------------------------
     DISCOVER:<id>
         Broadcast periódico de presença. Todos os postes
         respondem actualizando a sua tabela de vizinhos.

     SPD:<id>:<vel>:<eta_ms>:<dist_m>
         Enviado ao vizinho direito quando radar detecta carro.
         O vizinho usa o ETA para sincronizar o acendimento.

     TC_INC:<id>:<vel>
         Enviado ao vizinho direito imediatamente após detecção.
         O vizinho incrementa Tc e acende a 100% antecipadamente.
         Velocidade negativa = sinal de T-- (carro passou).

     STATUS:<id>:<estado>
         Propaga estado do poste: "OK", "FAIL", "SAFE", "AUTO".
         Usado para detectar falhas e activar modo autónomo.

     MASTER_CLAIM:<id>
         Enviado pelo poste com POST_POSITION=0 ao voltar online.
         O vizinho direito cede liderança e propaga em cadeia.

   Estrutura neighbor_t:
   ---------------------
     Compatível com comm_manager.h — campos .id, .status,
     .active e .position são necessários para a lógica da FSM.

   Dependências:
   -------------
   - system_config.h : UDP_PORT, MAX_NEIGHBORS, DISCOVER_INTERVAL_MS,
                       NEIGHBOR_TIMEOUT_MS, POSTE_ID, POST_POSITION
   - lwip            : sockets UDP
   - esp_timer       : timestamps de expiração
   - freertos        : task UDP

   Alterações v2.1 → v3.0:
   ------------------------
   1. neighbor_t expandido: .id, .position, .status, .active
   2. Adicionadas funções de envio: send_spd, send_tc_inc,
      send_status, send_master_claim
   3. udp_manager_discover() exposta publicamente
   4. udp_manager_get_neighbor_by_pos() adicionada
   5. neighbor_status_t definida neste módulo

============================================================ */

#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================
   ESTADO DO VIZINHO
============================================================ */
typedef enum {
    NEIGHBOR_OK      = 0,   /* Online e a responder          */
    NEIGHBOR_OFFLINE = 1,   /* Sem resposta > NEIGHBOR_TIMEOUT_MS */
    NEIGHBOR_FAIL    = 2,   /* Reportou falha via STATUS     */
    NEIGHBOR_SAFE    = 3,   /* Em modo seguro (radar falhou) */
    NEIGHBOR_AUTO    = 4,   /* Em modo autónomo (sem UDP)    */
} neighbor_status_t;

/* ============================================================
   ESTRUTURA DE VIZINHO
   Compatível com comm_manager.h
============================================================ */
typedef struct {
    char              ip[16];       /* Endereço IP "x.x.x.x\0"     */
    int               id;           /* ID do poste vizinho          */
    int               position;     /* Posição na cadeia            */
    neighbor_status_t status;       /* Estado actual do vizinho     */
    bool              active;       /* true se entrada válida       */
    bool              discover_ok;  /* true após DISCOVER confirmado —
                                       timeout só actua quando true  */
    uint32_t          last_seen;    /* Timestamp última recepção ms */
} neighbor_t;

/* ============================================================
   INICIALIZAÇÃO
============================================================ */

/**
 * @brief Inicializa socket UDP, task de recepção e DISCOVER.
 *        Deve ser chamada após Wi-Fi com IP obtido.
 * @return true se inicializado com sucesso, false em caso de erro
 */
bool udp_manager_init(void);

/* ============================================================
   ENVIO DE MENSAGENS
============================================================ */

/**
 * @brief Envia DISCOVER broadcast — anuncia presença a todos.
 *        Chamada periodicamente pela task interna.
 *        Pode ser chamada externamente pelo comm_manager.
 */
void udp_manager_discover(void);

/**
 * @brief Envia SPD ao vizinho direito com velocidade e ETA.
 * @param ip      IP do destinatário
 * @param speed   Velocidade do carro (km/h)
 * @param eta_ms  Tempo estimado de chegada (ms)
 * @param dist_m  Distância entre postes (m)
 * @return true se enviado com sucesso
 */
bool udp_manager_send_spd(const char *ip,
                           float       speed,
                           uint32_t    eta_ms,
                           uint32_t    dist_m);

/**
 * @brief Envia TC_INC ao vizinho indicado.
 *        Velocidade positiva = carro a caminho (Tc++).
 *        Velocidade negativa = carro passou (T--).
 * @param ip    IP do destinatário
 * @param speed Velocidade (negativa = passou)
 * @return true se enviado com sucesso
 */
bool udp_manager_send_tc_inc(const char *ip, float speed);

/**
 * @brief Envia STATUS ao vizinho indicado.
 * @param ip     IP do destinatário
 * @param status Estado a propagar
 * @return true se enviado com sucesso
 */
bool udp_manager_send_status(const char *ip,
                              neighbor_status_t status);

/**
 * @brief Envia MASTER_CLAIM ao vizinho indicado.
 *        Usado pelo poste pos=0 ao voltar online.
 * @param ip IP do destinatário
 * @return true se enviado com sucesso
 */
bool udp_manager_send_master_claim(const char *ip);

/* ============================================================
   CONSULTA DE VIZINHOS
============================================================ */

/**
 * @brief Preenche nebL e nebR com os IPs dos vizinhos.
 *        "---" se não houver vizinho conhecido.
 *        Mantém compatibilidade com main.c v2.4.
 * @param nebL Buffer min 16 bytes — vizinho esquerdo
 * @param nebR Buffer min 16 bytes — vizinho direito
 */
void udp_manager_get_neighbors(char *nebL, char *nebR);

/**
 * @brief Devolve ponteiro para vizinho com a posição indicada.
 * @param position Posição na cadeia a procurar
 * @return Ponteiro para neighbor_t ou NULL se não encontrado
 */
neighbor_t *udp_manager_get_neighbor_by_pos(int position);

/**
 * @brief Preenche lista com todos os vizinhos activos.
 * @param list  Array de destino
 * @param max   Tamanho máximo do array
 * @return Número de vizinhos activos copiados
 */
size_t udp_manager_get_all_neighbors(neighbor_t *list, size_t max);

/* ============================================================
   CALLBACKS — chamados pela task UDP ao receber mensagens
   Implementados na state_machine para processar eventos.
============================================================ */

/**
 * @brief Chamado ao receber TC_INC com velocidade positiva.
 *        Implementado em state_machine.c.
 */
void on_tc_inc_received(float speed);

/**
 * @brief Chamado ao receber TC_INC com velocidade negativa.
 *        Implementado em state_machine.c.
 */
void on_prev_passed_received(void);

/**
 * @brief Chamado ao receber SPD.
 *        Implementado em state_machine.c.
 */
void on_spd_received(float speed, uint32_t eta_ms);

/**
 * @brief Chamado ao receber MASTER_CLAIM.
 *        Implementado em state_machine.c.
 */
void on_master_claim_received(int from_id);

#endif /* UDP_MANAGER_H */