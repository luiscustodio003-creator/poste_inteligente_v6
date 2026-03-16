/* ============================================================
   POST CONFIG — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Gere a configuracao persistente do poste na flash NVS.
   No primeiro arranque, popula a NVS com os valores de
   POSTE_ID e POSTE_NAME definidos em system_config.h.
============================================================ */

#include "post_config.h"
#include "system_config.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "POST_CFG";

/* Namespace NVS exclusivo deste modulo */
#define NVS_NAMESPACE   "post_config"

/* Estado interno do modulo */
static post_config_t s_post = {0};

/* ============================================================
   post_config_init
   -----------------------------------------------------------
   Abre o namespace NVS e carrega ID e nome do poste.
   Se os valores nao existirem (primeiro flash), usa os
   defaults de system_config.h e guarda-os na NVS para
   que arranques futuros nao dependam do system_config.h.
   Pre-requisito: nvs_flash_init() ja chamado em app_main().
   ============================================================ */
void post_config_init(void)
{
    memset(&s_post, 0, sizeof(s_post));

    nvs_handle_t handle;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        uint32_t id       = POSTE_ID;           /* default do system_config.h */
        size_t   name_len = POST_NAME_MAX_LEN;

        /* Tenta carregar ID guardado anteriormente */
        nvs_get_u32(handle, "post_id", &id);

        /* Tenta carregar nome -- se nao existir usa POSTE_NAME */
        if (nvs_get_str(handle, "post_name",
                        s_post.name, &name_len) != ESP_OK)
        {
            /* Primeiro arranque -- popula NVS com valores de system_config.h */
            strncpy(s_post.name, POSTE_NAME, POST_NAME_MAX_LEN - 1);
            s_post.name[POST_NAME_MAX_LEN - 1] = '\0';

            nvs_set_str(handle, "post_name", s_post.name);
            nvs_set_u32(handle, "post_id",   (uint32_t)POSTE_ID);
            nvs_commit(handle);

            ESP_LOGI(TAG, "Primeiro arranque | NVS populada | ID=%d NAME=%s",
                     POSTE_ID, POSTE_NAME);
        }

        s_post.id = (uint8_t)id;
        nvs_close(handle);

        ESP_LOGI(TAG, "Config carregada | ID=%d | NAME=%s",
                 s_post.id, s_post.name);
    }
    else
    {
        /* NVS nao acessivel -- usa valores de system_config.h em RAM */
        s_post.id = (uint8_t)POSTE_ID;
        strncpy(s_post.name, POSTE_NAME, POST_NAME_MAX_LEN - 1);
        s_post.name[POST_NAME_MAX_LEN - 1] = '\0';

        ESP_LOGW(TAG, "NVS nao acessivel | usando system_config.h");
    }
}

/* Retorna ID do poste */
uint8_t post_get_id(void)
{
    return s_post.id;
}

/* Retorna nome do poste */
const char *post_get_name(void)
{
    return s_post.name;
}

/* Define novo ID e persiste na NVS */
void post_set_id(uint8_t id)
{
    s_post.id = id;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_u32(handle, "post_id", (uint32_t)id);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "ID guardado: %d", id);
    }
}

/* Define novo nome e persiste na NVS */
void post_set_name(const char *name)
{
    if (!name) return;

    strncpy(s_post.name, name, POST_NAME_MAX_LEN - 1);
    s_post.name[POST_NAME_MAX_LEN - 1] = '\0';

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_str(handle, "post_name", s_post.name);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Nome guardado: %s", s_post.name);
    }
}