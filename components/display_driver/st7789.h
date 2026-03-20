/* ============================================================
   ST7789 — DECLARAÇÃO
   ------------------------------------------------------------
   @file      st7789.h
   @brief     Driver SPI para display TFT ST7789 240x240
   @version   2.1
   @date      2026-03-19

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Interface pública do driver SPI para o controlador ST7789.
   Fornece inicialização, preenchimento de ecrã e escrita
   de bitmaps RGB565. Utilizado exclusivamente pelo
   display_manager através do callback LVGL st7789_flush_cb().

   Dependências:
   -------------
   - hw_config.h  : pinos GPIO (LCD_PIN_*) e resolução (LCD_*_RES)
   - driver       : spi_master, gpio (ESP-IDF)
   - freertos     : vTaskDelay

============================================================ */

#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>

/**
 * @brief Inicializa SPI, efectua reset hardware e executa
 *        a sequência de inicialização do ST7789.
 *        Deve ser chamada antes de qualquer outra função.
 */
void st7789_init(void);

/**
 * @brief Preenche o ecrã inteiro com uma cor sólida.
 * @param color Cor no formato RGB565 (big-endian)
 */
void st7789_fill(uint16_t color);

/**
 * @brief Escreve um bitmap RGB565 numa região do ecrã.
 * @param x      Coluna inicial (0 a LCD_H_RES-1)
 * @param y      Linha inicial  (0 a LCD_V_RES-1)
 * @param w      Largura em pixels
 * @param h      Altura em pixels
 * @param data   Array de pixels RGB565 (w * h elementos)
 */
void st7789_draw_bitmap(int x, int y, int w, int h,
                        const uint16_t *data);

#endif /* ST7789_H */