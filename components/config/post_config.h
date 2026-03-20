/* ============================================================
   POST CONFIG — DECLARAÇÃO
   ------------------------------------------------------------
   @file      post_config.h
   @brief     Gestão da configuração persistente do poste na NVS
   @version   2.1
   @date      2026-03-19

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere a configuração persistente do poste na flash NVS.
   No primeiro arranque após flash, popula a NVS com os
   valores de POSTE_ID e POSTE_NAME definidos em system_config.h.
   Em arranques subsequentes carrega os valores da NVS,
   garantindo que ID e nome sobrevivem a cortes de energia.

   Pré-requisito:
   --------------
   nvs_flash_init() deve ser chamado em app_main() ANTES
   de post_config_init(). Este módulo não inicializa a NVS.

   Dependências:
   -------------
   - system_config.h  : valores por omissão (POSTE_ID, POSTE_NAME)
   - nvs_flash        : inicializada em app_main antes deste módulo
   - esp_log          : sistema de logging do ESP-IDF


============================================================ */

#ifndef POST_CONFIG_H
#define POST_CONFIG_H

#include <stdint.h>

/* Comprimento máximo do nome do poste (inclui terminador nulo) */
#define POST_NAME_MAX_LEN   33

/* ============================================================
   Estrutura de configuração do poste
============================================================ */
typedef struct {
    uint8_t id;                     /* Identificador numérico único  */
    char    name[POST_NAME_MAX_LEN];/* Nome legível do poste         */
} post_config_t;

/* ============================================================
   API PÚBLICA
============================================================ */

/**
 * @brief Inicializa a configuração do poste.
 *        Carrega ID e nome da NVS. Se a chave não existir
 *        (primeiro flash), usa os valores de system_config.h
 *        e persiste-os na NVS para arranques futuros.
 */
void        post_config_init(void);

/**
 * @brief Retorna o ID actual do poste (lido da NVS ou default).
 * @return uint8_t ID do poste
 */
uint8_t     post_get_id(void);

/**
 * @brief Retorna o nome actual do poste (lido da NVS ou default).
 * @return Ponteiro para string com o nome (não deve ser modificado)
 */
const char *post_get_name(void);

/**
 * @brief Define novo ID e persiste imediatamente na NVS.
 * @param id Novo identificador numérico
 */
void        post_set_id(uint8_t id);

/**
 * @brief Define novo nome e persiste imediatamente na NVS.
 * @param name Nova string de nome (truncada a POST_NAME_MAX_LEN-1)
 */
void        post_set_name(const char *name);

#endif /* POST_CONFIG_H */