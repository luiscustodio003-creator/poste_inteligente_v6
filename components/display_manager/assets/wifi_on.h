/* ============================================================
   WIFI ON ICON — DECLARAÇÃO
   ------------------------------------------------------------
   @file      wifi_on.h
   @brief     Declaração do descriptor de imagem LVGL para o
              ícone Wi-Fi ligado (alpha 1-bit, 48×48 px)
   @version   1.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Expõe o descriptor lv_img_dsc_t gerado em wifi_on.c
   para uso directo com lv_img_set_src() do LVGL 8.x.

   Formato da imagem:
   ------------------
   - Tipo    : LV_IMG_CF_ALPHA_1BIT (máscara binária, 1 bit/pixel)
   - Tamanho : 48 × 48 pixels
   - Dados   : 288 bytes (48 linhas × 6 bytes por linha)
               Cada linha tem ceil(48/8) = 6 bytes exactos.
               Sem padding — 48 é múltiplo de 8.

   Diferença para wifi_off:
   ------------------------
   - wifi_off : 50×50 px → 7 bytes/linha (50 bits + 6 bits padding)
   - wifi_on  : 48×48 px → 6 bytes/linha (48 bits, sem padding)

   Como usar:
   ----------
     #include "wifi_on.h"
     lv_img_set_src(img_widget, &wifi_on);

   Recoloração (para mudar a cor do ícone em runtime):
   ---------------------------------------------------
     lv_obj_set_style_img_recolor(widget, lv_color_make(0,255,0), 0);
     lv_obj_set_style_img_recolor_opa(widget, LV_OPA_COVER, 0);

   Dependências:
   -------------
   - lvgl 8.x  (lv_img_dsc_t, LV_IMG_CF_ALPHA_1BIT)
============================================================ */

#ifndef WIFI_ON_H
#define WIFI_ON_H

/* Inclusão compatível com projectos que têm lvgl.h na raiz
   ou dentro de uma pasta lvgl/                             */
#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

/* Descriptor da imagem — definido em wifi_on.c */
extern const lv_img_dsc_t wifi_on;

#endif /* WIFI_ON_H */