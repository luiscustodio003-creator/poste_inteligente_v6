/* ============================================================
   POST CONFIG — IMPLEMENTACAO
   ------------------------------------------------------------
   @file      post_config.c
   @brief     Gestao da configuracao persistente do poste
   @version   2.1
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alteracoes (v2.0 -> v2.1):
   --------------------------
   1. CORRECCAO: valores por defeito passaram a usar POSTE_ID
      e POSTE_NAME de system_config.h em vez de constantes
      hardcoded (ID=0, NAME="POSTE").
      Na versao anterior, no primeiro arranque apos flash, o
      display mostrava "POSTE" em vez de "POSTE 01" ou "POSTE 02"
      porque a NVS estava vazia e os defaults nao coincidiam
      com o system_config.h.
      Agora no primeiro arranque a NVS e populada automaticamente
      com os valores correctos do system_config.h.

   Pre-requisito:
   --------------
   nvs_flash_init() deve ser chamado em app_main() antes de
   post_config_init().

   Dependencias:
   -------------
   - post_config.h
   - system_config.h  (POSTE_ID, POSTE_NAME)
   - nvs, esp_log
============================================================ */

#include "post_config.h"
#include "system_config.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

#define NVS_NAMESPACE   "post_config"

static const char    *TAG    = "POST_CFG";
static post_config_t  s_post = {0};

/* ============================================================
   INICIALIZACAO
   -----------------------------------------------------------
   Tenta carregar ID e nome da NVS.
   Se nao existirem dados (primeiro arranque apos flash),
   usa POSTE_ID e POSTE_NAME de system_config.h como valores
   por defeito e persiste-os imediatamente na NVS.
   ============================================================ */

void post_config_init(void)
{
    memset(&s_post, 0, sizeof(s_post));

    nvs_handle_t handle;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        uint32_t id       = POSTE_ID;           /* default: system_config.h */
        size_t   name_len = POST_NAME_MAX_LEN;

        /* Carrega ID -- se nao existir usa POSTE_ID */
        nvs_get_u32(handle, "post_id", &id);

        /* Carrega nome -- se nao existir usa POSTE_NAME */
        if (nvs_get_str(handle, "post_name",
                        s_post.name, &name_len) != ESP_OK)
        {
            strncpy(s_post.name, POSTE_NAME, POST_NAME_MAX_LEN);
            s_post.name[POST_NAME_MAX_LEN - 1] = '\0';

            /* Persiste o nome por defeito na NVS imediatamente */
            nvs_set_str(handle, "post_name", s_post.name);
            nvs_set_u32(handle, "post_id", (uint32_t)POSTE_ID);
            nvs_commit(handle);

            ESP_LOGI(TAG,
                     "Primeiro arranque -- NVS populada | ID=%d | NAME=%s",
                     POSTE_ID, POSTE_NAME);
        }

        s_post.id = (uint8_t)id;

        nvs_close(handle);

        ESP_LOGI(TAG, "Config carregada | ID=%d | NAME=%s",
                 s_post.id, s_post.name);
    }
    else
    {
        /* NVS nao acessivel -- usa valores de system_config.h */
        s_post.id = (uint8_t)POSTE_ID;
        strncpy(s_post.name, POSTE_NAME, POST_NAME_MAX_LEN);
        s_post.name[POST_NAME_MAX_LEN - 1] = '\0';

        ESP_LOGW(TAG,
                 "NVS nao acessivel -- valores de system_config.h | ID=%d | NAME=%s",
                 s_post.id, s_post.name);
    }
}

/* ============================================================
   GETTERS
   ============================================================ */

uint8_t post_get_id(void)
{
    return s_post.id;
}

const char *post_get_name(void)
{
    return s_post.name;
}

/* ============================================================
   SETTERS — Persistem na NVS imediatamente
   ============================================================ */

void post_set_id(uint8_t id)
{
    s_post.id = id;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_u32(handle, "post_id", (uint32_t)id);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "ID guardado na NVS: %d", id);
    }
}

void post_set_name(const char *name)
{
    if (!name) return;

    strncpy(s_post.name, name, POST_NAME_MAX_LEN);
    s_post.name[POST_NAME_MAX_LEN - 1] = '\0';

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_str(handle, "post_name", s_post.name);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Nome guardado na NVS: %s", s_post.name);
    }
}