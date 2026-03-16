/* ============================================================
   DISPLAY MANAGER — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Gestao do display TFT ST7789 240x240 com LVGL 8.3.
   Mostra em tempo real todas as informacoes do poste:
   estado da linha, vizinhos ESQ/DIR, contadores T/Tc,
   brilho actual, velocidade e estado FSM.

   Layout do display 240x240:
   --------------------------
   POSTE 01  [MASTER]          <- y=8  nome + estado linha
   IP: 192.168.4.1             <- y=26 IP com prefixo AP/IP
   ─────────────────────────   <- separador y=40
   ESQ: P01 OK  | DIR: P03 OK  <- y=52 vizinhos esq/dir
   ─────────────────────────   <- separador y=65
   T:2  Tc:1  VEL: 72 km/h    <- y=77 contadores + velocidade
   ─────────────────────────   <- separador y=90
   P02 .4.2 OK   72km  2s      <- y=102 vizinho 1
   P03 .4.3 OFF  --km 15s      <- y=122 vizinho 2
   P04 .4.4 SAFE 50km  8s      <- y=142 vizinho 3
   P05 .4.5 OK   45km  1s      <- y=162 vizinho 4
   ─────────────────────────   <- separador y=178
   LUZ: 100%  [LIGHT ON]       <- y=192 brilho + estado FSM
   ─────────────────────────   <- separador y=208

   Dependencias:
   -------------
   - st7789.h, lvgl.h
   - udp_manager.h (neighbor_t)
   - freertos, esp_timer
============================================================ */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "udp_manager.h"

/* Inicializa display, LVGL e cria lvgl_task */
void display_manager_start(void);

/* Actualiza prefixo IP: "AP: x.x.x.x" ou "IP: x.x.x.x" */
void display_manager_set_ap_mode(bool ap_active, const char *ip);

/* Actualiza linha de vizinhos ESQ e DIR */
void display_manager_update_neighbors_lr(const char *esq_str,
                                          const char *dir_str);

/* Actualiza contadores T, Tc e velocidade */
void display_manager_update_traffic(int T, int Tc, int vel_kmh);

/* Actualiza tabela de vizinhos completa */
void display_manager_update_neighbors(const neighbor_t *neighbors,
                                       size_t            count);

/* Actualiza nome do poste e estado da linha */
void display_manager_update_info(const char *poste_name,
                                  const char *estado_linha);

/* Actualiza brilho actual e estado FSM com cor dinamica */
void display_manager_update_brightness(uint8_t     brightness,
                                        const char *state_name);

#endif /* DISPLAY_MANAGER_H */