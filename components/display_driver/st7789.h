/* ============================================================
   ST7789 — INTERFACE
   ------------------------------------------------------------
   @file      st7789.h
   @brief     Driver SPI para display TFT ST7789 (240x240)
   @version   4.0
   @date      2026-03-23

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Interface do driver ST7789 para comunicação SPI.
   Totalmente compatível com LVGL (flush callback).

   Dependências:
   -------------
   - hw_config.h  : definição de pinos LCD_PIN_*
   - driver/spi_master
   ============================================================ */

#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>
#include <stdbool.h>
#include "hw_config.h"

/* ============================================================
   API PÚBLICA
============================================================ */

/* Inicializa o display */
void st7789_init(void);

/* Define janela activa de escrita */
void st7789_set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1);

/* Escreve bitmap RGB565 (usado pelo LVGL) */
void st7789_draw_bitmap(uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        const uint16_t *data);

/* Controla backlight */
void st7789_backlight(bool on);

#endif /* ST7789_H */