#include "wifi_icon.h"

/* ============================================================
   WIFI ICON IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      wifi_icon.c
   @brief     Ícone de WiFi para LVGL 8.x
   @version   1.0
   @date      2026-03-14

   Descrição:
   ----------
   Define o array de dados da imagem WiFi e o descriptor
   lv_img_dsc_t compatível com LVGL 8.x.

   Observações:
   ------------
   - Para imagens com transparência usar LV_IMG_CF_TRUE_COLOR_ALPHA
   - Dados da imagem podem ser gerados com lvgl/lv_img_conv.py
============================================================ */

/* Dados da imagem (RGB565) */
static const uint8_t wifi_icon_map[] = {
    /* Exemplo de bitmap 16x16, substituir pelo bitmap real */
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/* Descriptor da imagem */
const lv_img_dsc_t wifi_icon_img = {
    .header.always_zero = 0,                  // LVGL interno
    .header.w = 20,                           // largura da imagem
    .header.h = 20,                           // altura da imagem
    .header.cf = LV_IMG_CF_TRUE_COLOR,        // formato de cor RGB565 sem alpha
    .data_size = sizeof(wifi_icon_map),       // tamanho em bytes do array
    .data = wifi_icon_map                      // ponteiro para os dados
};