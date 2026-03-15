/* ============================================================
   POST CONFIG — DECLARAÇÃO
   ------------------------------------------------------------
   @file      post_config.h
   @brief     Gestão da configuração persistente do poste
   @version   2.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Carrega e persiste o ID e nome do poste na NVS (flash).
   Valores sobrevivem a rearranques e cortes de energia.

   PRÉ-REQUISITO:
   --------------
   nvs_flash_init() deve ser chamado em app_main() antes de
   post_config_init(). Este módulo não inicializa a NVS.

   Utilização típica:
   ------------------
     // Em app_main():
     nvs_flash_init();           // uma vez, antes de tudo
     post_config_init();         // carrega da NVS
     ESP_LOGI("", "%s", post_get_name());

     // Para alterar e persistir:
     post_set_id(2);
     post_set_name("POSTE 02");

   Dependências:
   -------------
   - nvs (já inicializada por app_main)
   - esp_log
============================================================ */

#ifndef POST_CONFIG_H
#define POST_CONFIG_H

#include <stdint.h>

#define POST_NAME_MAX_LEN   32

/* Estrutura de configuração do poste */
typedef struct
{
    uint8_t id;                     /* ID único do poste (0-255)  */
    char    name[POST_NAME_MAX_LEN]; /* Nome para display e logs   */
} post_config_t;

/* -----------------------------------------------------------
   post_config_init — Carrega configuração da NVS

   PRÉ-REQUISITO: nvs_flash_init() já chamado em app_main().
   Se a NVS não tiver dados, usa valores por defeito:
     ID=0, NAME="POSTE"
   ----------------------------------------------------------- */
void post_config_init(void);

/* Retorna ID actual do poste */
uint8_t post_get_id(void);

/* Retorna nome actual do poste */
const char *post_get_name(void);

/* Define novo ID e persiste na NVS */
void post_set_id(uint8_t id);

/* Define novo nome e persiste na NVS */
void post_set_name(const char *name);

#endif /* POST_CONFIG_H */