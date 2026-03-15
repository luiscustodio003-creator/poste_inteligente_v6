/* ============================================================
   DISPLAY MANAGER -- DECLARACAO
   ------------------------------------------------------------
   @file      display_manager.h
   @brief     Gestao do display ST7789 com LVGL 8.3
   @version   5.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alteracoes (v4.0 -> v5.0):
   --------------------------
   1. Adicionada display_manager_update_brightness() -- mostra
      o brilho actual da luminaria em percentagem no display.
      Essencial para o operador saber o estado da luminaria:
        IDLE      -> LIGHT_MIN  (10%)
        DETECTION -> 55%        ((100+10)/2)
        LIGHT_ON  -> LIGHT_MAX  (100%)
        TIMEOUT   -> LIGHT_MIN  (10%)
        SAFE_MODE -> LIGHT_SAFE (50%) ou 100% se veiculo
   2. Label de brilho com cor dinamica:
        verde  = brilho maximo (100%)
        ambar  = brilho intermedio (11-99%)
        cinza  = brilho minimo (<=10%)

   Layout do display 240x240 actualizado:
   ----------------------------------------
   [icone wifi]  [NOME POSTE]    <- y=5/10
   IP: x.x.x.x                  <- y=35
   VIZINHOS: N                   <- y=55
   ──────────────────────────    <- separador y=73
   P01 .4.2 OK   52km  2s        <- vizinho 1 y=80
   P02 .4.3 OFF  --km 15s        <- vizinho 2 y=102
   P03 .4.4 SAFE 30km  8s        <- vizinho 3 y=124
   P04 .4.5 OK   45km  1s        <- vizinho 4 y=146
   ──────────────────────────    <- separador y=168
   LUZ: 50%  [SAFE MODE]         <- y=185  NOVO
   VEL: 52 km/h                  <- y=210

   Dependencias:
   -------------
   - st7789, lvgl 8.3, wifi_off, wifi_on
   - udp_manager.h (neighbor_t, neighbor_status_t)
   - FreeRTOS, lv_conf.h
============================================================ */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include "udp_manager.h"

/* Inicializa display, LVGL e todos os widgets */
void display_manager_init(void);

/* Troca icone Wi-Fi: true=wifi_on(verde) / false=wifi_off(vermelho) */
void display_manager_set_wifi(bool connected);

/* Sinaliza modo AP: true="AP: ip" / false="IP: ip" */
void display_manager_set_ap_mode(bool ap_active, const char *ip);

/* Actualiza labels principais (ip ignorado -- gerido por set_ap_mode) */
void display_manager_update_info(const char *poste,
                                 const char *ip,
                                 int         vizinhos,
                                 int         vel);

/* Actualiza tabela de vizinhos:
   ID | IP | estado (OK/OFF/SAFE) | velocidade | tempo */
void display_manager_update_neighbors(const neighbor_t *neighbors,
                                      size_t            count);

/* Actualiza brilho da luminaria no display.
   brightness: 0-100 (%)
   state_name: nome do estado FSM actual ("IDLE", "LIGHT_ON",
               "SAFE MODE", "DETECTION", "TIMEOUT")
   Cor do label: verde=100%, ambar=11-99%, cinza<=10%          */
void display_manager_update_brightness(uint8_t     brightness,
                                       const char *state_name);

/* Inicializa display e cria task LVGL */
void display_manager_start(void);

#endif /* DISPLAY_MANAGER_H */