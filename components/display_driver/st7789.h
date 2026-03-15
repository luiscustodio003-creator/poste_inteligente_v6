/* ============================================================
   ST7789 -- DECLARACAO
   ------------------------------------------------------------
   @file      st7789.h
   @brief     Driver SPI para display TFT ST7789 240x240
   @version   2.1
   @date      2026-03-15

   Alteracoes (v2.0 -> v2.1):
   --------------------------
   1. CORRECCAO: removida declaracao de st7789_send_color()
      que existia no .h mas nao estava implementada no .c,
      causaria erro de linker se chamada.

   Dependencias:
   -------------
   - hw_config.h (pinos GPIO e resolucao)
============================================================ */

#ifndef ST7789_H
#define ST7789_H

#include <stdint.h>

/* Inicializa SPI, reset hardware e sequencia de init ST7789 */
void st7789_init(void);

/* Preenche ecra inteiro com cor RGB565 */
void st7789_fill(uint16_t color);

/* Desenha bitmap RGB565 em (x,y) com dimensoes (w,h) */
void st7789_draw_bitmap(int x, int y, int w, int h,
                        const uint16_t *data);

#endif /* ST7789_H */