/* ============================================================
   POST CONFIG — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      post_config.c
   @brief     Gestão da configuração persistente do poste na NVS
   @version   2.1
   @date      2026-03-19

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere a configuração persistente do poste na flash NVS do ESP32.
   No primeiro arranque após flash, popula a NVS com os valores de
   POSTE_ID e POSTE_NAME definidos em system_config.h. Em arranques
   subsequentes carrega da NVS, garantindo que o nome e ID
   sobrevivem a cortes de energia e rearranques.

============================================================ */

#include "post_config.h"
#include "system_config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

/* Etiqueta de log deste módulo */
static const char *TAG = "POST_CFG";

/* Namespace NVS exclusivo deste módulo */
#define NVS_NAMESPACE   "post_config"

/* Estado interno do módulo — único ponto de verdade em RAM */
static post_config_t s_post = {0};

/* ============================================================
   _persist_id
   ------------------------------------------------------------
   Função interna: abre NVS, guarda o ID e fecha.
   Usada por post_config_init() e post_set_id().
============================================================ */
static void _persist_id(uint8_t id)
{
    nvs_handle_t handle;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_u32(handle, "post_id", (uint32_t)id);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "ID guardado na NVS: %d", id);
    }
    else
    {
        ESP_LOGW(TAG, "Não foi possível guardar ID na NVS");
    }
}

/* ============================================================
   _persist_name
   ------------------------------------------------------------
   Função interna: abre NVS, guarda o nome e fecha.
   Usada por post_config_init() e post_set_name().
============================================================ */
static void _persist_name(const char *name)
{
    nvs_handle_t handle;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_str(handle, "post_name", name);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Nome guardado na NVS: %s", name);
    }
    else
    {
        ESP_LOGW(TAG, "Não foi possível guardar nome na NVS");
    }
}

/* ============================================================
   post_config_init
   ------------------------------------------------------------
   Abre o namespace NVS e carrega ID e nome do poste.
   Se os valores não existirem (primeiro flash), usa os
   defaults de system_config.h e guarda-os na NVS para
   que arranques futuros não dependam do system_config.h.

   Pré-requisito: nvs_flash_init() já chamado em app_main().
============================================================ */
void post_config_init(void)
{
    /* Limpa estrutura interna */
    memset(&s_post, 0, sizeof(s_post));

    nvs_handle_t handle;
    bool primeiro_arranque = false;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK)
    {
        /* --- Carregar ID --- */
        uint32_t id = (uint32_t)POSTE_ID;   /* valor por omissão */
        nvs_get_u32(handle, "post_id", &id);
        s_post.id = (uint8_t)id;

        /* --- Carregar nome ---
           nvs_get_str exige que name_len contenha o tamanho máximo
           do buffer (incluindo terminador) antes da chamada.        */
        size_t name_len = sizeof(s_post.name);
        esp_err_t err = nvs_get_str(handle, "post_name",
                                    s_post.name, &name_len);

        if (err != ESP_OK)
        {
            /* Primeiro arranque — popula NVS com valores de system_config.h */
            primeiro_arranque = true;
            strncpy(s_post.name, POSTE_NAME, sizeof(s_post.name) - 1);
            s_post.name[sizeof(s_post.name) - 1] = '\0';

            nvs_set_str(handle, "post_name", s_post.name);
            nvs_set_u32(handle, "post_id",   (uint32_t)POSTE_ID);
            nvs_commit(handle);

            ESP_LOGI(TAG, "Primeiro arranque | NVS populada | ID=%d NAME=%s",
                     POSTE_ID, POSTE_NAME);
        }

        nvs_close(handle);

        if (primeiro_arranque)
        {
            ESP_LOGI(TAG, "Defaults aplicados | ID=%d | NAME=%s",
                     s_post.id, s_post.name);
        }
        else
        {
            ESP_LOGI(TAG, "Config carregada da NVS | ID=%d | NAME=%s",
                     s_post.id, s_post.name);
        }
    }
    else
    {
        /* NVS não acessível — usa valores de system_config.h apenas em RAM */
        s_post.id = (uint8_t)POSTE_ID;
        strncpy(s_post.name, POSTE_NAME, sizeof(s_post.name) - 1);
        s_post.name[sizeof(s_post.name) - 1] = '\0';

        ESP_LOGW(TAG, "NVS não acessível | usando valores de system_config.h");
    }
}

/* ============================================================
   post_get_id
   ------------------------------------------------------------
   Retorna o ID actual do poste.
============================================================ */
uint8_t post_get_id(void)
{
    return s_post.id;
}

/* ============================================================
   post_get_name
   ------------------------------------------------------------
   Retorna o nome actual do poste como string constante.
   O ponteiro é válido durante toda a vida do programa.
============================================================ */
const char *post_get_name(void)
{
    return s_post.name;
}

/* ============================================================
   post_set_id
   ------------------------------------------------------------
   Define novo ID em RAM e persiste imediatamente na NVS.
============================================================ */
void post_set_id(uint8_t id)
{
    s_post.id = id;
    _persist_id(id);
}

/* ============================================================
   post_set_name
   ------------------------------------------------------------
   Define novo nome em RAM e persiste imediatamente na NVS.
   Rejeita ponteiro nulo. Trunca a POST_NAME_MAX_LEN-1 chars.
============================================================ */
void post_set_name(const char *name)
{
    if (!name)
    {
        ESP_LOGW(TAG, "post_set_name: ponteiro nulo ignorado");
        return;
    }

    strncpy(s_post.name, name, sizeof(s_post.name) - 1);
    s_post.name[sizeof(s_post.name) - 1] = '\0';

    _persist_name(s_post.name);
}