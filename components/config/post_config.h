/* ============================================================
   POST CONFIG — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Gestao da configuracao persistente do poste na NVS.
   No primeiro arranque apos flash, popula a NVS com os
   valores de POSTE_ID e POSTE_NAME de system_config.h.
   Em arranques subsequentes carrega da NVS.

   Dependencias:
   -------------
   - system_config.h (POSTE_ID, POSTE_NAME)
   - nvs_flash (inicializada em app_main antes deste modulo)
============================================================ */

#ifndef POST_CONFIG_H
#define POST_CONFIG_H

#include <stdint.h>

#define POST_NAME_MAX_LEN   32

/* Estrutura de configuracao do poste */
typedef struct {
    uint8_t id;
    char    name[POST_NAME_MAX_LEN];
} post_config_t;

/* Inicializa config -- carrega NVS ou usa defaults de system_config.h */
void        post_config_init(void);

/* Getters */
uint8_t     post_get_id(void);
const char *post_get_name(void);

/* Setters -- persistem na NVS imediatamente */
void        post_set_id(uint8_t id);
void        post_set_name(const char *name);

#endif /* POST_CONFIG_H */