/* ============================================================
   UDP MANAGER — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Camada de transporte UDP entre postes inteligentes.
   Implementa o protocolo completo de descoberta, propagacao
   de velocidade com ETA, gestao de contadores T/Tc,
   lideranca de linha (MASTER_CLAIM) e estado (STATUS).

   Protocolo de mensagens (texto simples, separado por ':'):
   ----------------------------------------------------------
   DISCOVER:<id>:<nome>:<pos>
     Broadcast de anuncio periodico.

   DISCOVER_ACK:<id>:<nome>:<pos>:<ip>
     Resposta unicast ao DISCOVER.

   SPD:<id>:<nome>:<pos>:<vel>:<eta_ms>:<dist_m>
     Velocidade + ETA para o proximo poste.
     Receptor incrementa Tc e agenda acendimento no ETA.

   TC_INC:<id>:<pos>:<vel>
     Incrementa Tc no vizinho -- carro a caminho.
     Enviado imediatamente apos deteccao pelo radar.

   ACK:<id>:<msg_type>
     Confirmacao de recepcao.

   STATUS:<id>:<pos>:<estado>
     Estado do poste: OK / OFFLINE / SAFE / MASTER

   MASTER_CLAIM:<id>:<pos>
     Reclama lideranca da linha. pos=0 tem prioridade maxima.
     Propagado em cadeia de poste em poste.

   Contadores T e Tc por poste (implementados em state_machine):
   -------------------------------------------------------------
   T[i]  = carros presentes na zona do poste i (radar confirmou)
   Tc[i] = carros a caminho do poste i (UDP informou, ainda nao chegaram)
   Luz 100% quando T>0 OU Tc>0
   Luz 10%  apenas quando T=0 E Tc=0 (apos TRAFIC_TIMEOUT_MS)

   Dependencias:
   -------------
   - system_config.h
   - post_config.h
   - wifi_manager.h
   - lwip/sockets, nvs, esp_timer
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
typedef enum {
    NEIGHBOR_OK        = 0, /* Activo e a comunicar normalmente   */
    NEIGHBOR_OFFLINE,       /* Sem resposta apos NEIGHBOR_TIMEOUT */
    NEIGHBOR_SAFE_MODE,     /* Online mas radar falhou -- 50%     */
    NEIGHBOR_MASTER         /* Assumiu lideranca da linha         */
} neighbor_status_t;

/* ===========================================================
   Estrutura de vizinho
   =========================================================== */
typedef struct {
    char              ip[MAX_IP_LEN]; /* IP do vizinho                  */
    int               id;             /* ID do poste vizinho            */
    int               pos;            /* Posicao na cadeia              */
    char              name[32];       /* Nome do poste                  */
    uint64_t          last_seen;      /* Timestamp ultimo contacto (ms) */
    float             last_speed;     /* Ultima velocidade recebida     */
    uint32_t          last_eta_ms;    /* Ultimo ETA recebido (ms)       */
    neighbor_status_t status;         /* Estado actual do vizinho       */
    bool              active;         /* Registo valido na tabela       */
} neighbor_t;

/* ===========================================================
   Estrutura de mensagem SPD recebida
   =========================================================== */
typedef struct {
    int      id;            /* ID do poste emissor             */
    int      pos;           /* Posicao do poste emissor        */
    float    speed;         /* Velocidade do veiculo (km/h)    */
    uint32_t eta_ms;        /* ETA para o proximo poste (ms)   */
    float    dist_m;        /* Distancia entre postes (m)      */
} spd_msg_t;

/* ===========================================================
   API — Inicializacao
   =========================================================== */

/* Cria socket UDP, faz bind na UDP_PORT, carrega NVS */
bool udp_manager_init(void);

/* Carrega tabela de vizinhos da NVS */
void udp_manager_load_neighbors_nvs(void);

/* Persiste tabela de vizinhos na NVS */
void udp_manager_save_neighbors_nvs(void);

/* ===========================================================
   API — Envio
   =========================================================== */

/* Envia mensagem para IP especifico */
bool udp_manager_send_to(const char *buf, size_t len, const char *ip);

/* Envia DISCOVER broadcast */
void udp_manager_discover(void);

/* Envia SPD com ETA calculado para vizinho directo */
bool udp_manager_send_spd(const char *dest_ip,
                           float       speed,
                           uint32_t    eta_ms,
                           float       dist_m);

/* Envia TC_INC -- incrementa Tc no vizinho imediatamente */
bool udp_manager_send_tc_inc(const char *dest_ip, float speed);

/* Envia ACK de confirmacao */
bool udp_manager_send_ack(const char *dest_ip, const char *msg_type);

/* Envia STATUS para vizinho */
bool udp_manager_send_status(const char *dest_ip, neighbor_status_t status);

/* Envia MASTER_CLAIM para vizinho directo */
bool udp_manager_send_master_claim(const char *dest_ip);

/* ===========================================================
   API — Recepcao e processamento
   =========================================================== */

/* Processa UMA mensagem recebida (nao bloqueante).
   Retorna tipo da mensagem processada ou UDP_MSG_NONE. */
typedef enum {
    UDP_MSG_NONE = 0,
    UDP_MSG_SPD,            /* Velocidade + ETA recebidos     */
    UDP_MSG_TC_INC,         /* Incrementar Tc (carro a caminho) */
    UDP_MSG_STATUS,         /* Estado de vizinho alterado     */
    UDP_MSG_MASTER_CLAIM,   /* Vizinho reclama lideranca      */
    UDP_MSG_DISCOVER,       /* Novo vizinho anunciou-se       */
    UDP_MSG_ACK             /* Confirmacao recebida           */
} udp_msg_type_t;

udp_msg_type_t udp_manager_process(spd_msg_t        *spd_out,
                                    neighbor_status_t *status_out,
                                    int               *src_id_out,
                                    int               *src_pos_out);

/* ===========================================================
   API — Tabela de vizinhos
   =========================================================== */

/* Copia vizinhos activos para list[]. Retorna contagem. */
size_t udp_manager_get_neighbors(neighbor_t *list, size_t max);

/* Retorna true se existir pelo menos um vizinho online */
bool udp_manager_has_active_neighbors(void);

/* Marca vizinhos sem contacto como OFFLINE */
void udp_manager_check_timeouts(void);

/* Retorna contagem de vizinhos registados */
size_t udp_manager_get_neighbor_count(void);

/* Retorna vizinho com posicao especifica (ou NULL) */
neighbor_t *udp_manager_get_neighbor_by_pos(int pos);

#endif /* UDP_MANAGER_H */