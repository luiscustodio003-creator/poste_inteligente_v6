/* ============================================================
   WIFI OFF ICON — DECLARAÇÃO
   ------------------------------------------------------------
   @file      wifi_off.h
   @brief     Declaração do descriptor de imagem LVGL para o
              ícone Wi-Fi desligado (alpha 1-bit, 50×50 px)
   @version   1.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Expõe o descriptor lv_img_dsc_t gerado em wifi_off.c
   para uso directo com lv_img_set_src() do LVGL 8.x.

   Formato da imagem:
   ------------------
   - Tipo    : LV_IMG_CF_ALPHA_1BIT (máscara binária, 1 bit/pixel)
   - Tamanho : 50 × 50 pixels
   - Dados   : 350 bytes (50 × 50 / 8 × 1 bit arredondado a byte)

   Como usar:
   ----------
     #include "wifi_off.h"
     lv_img_set_src(img_widget, &wifi_off);

   Recoloração (para mudar a cor do ícone em runtime):
   ---------------------------------------------------
     lv_obj_set_style_img_recolor(widget, lv_color_make(255,0,0), 0);
     lv_obj_set_style_img_recolor_opa(widget, LV_OPA_COVER, 0);

   Dependências:
   -------------
   - lvgl 8.x  (lv_img_dsc_t, LV_IMG_CF_ALPHA_1BIT)
============================================================ */

#ifndef WIFI_OFF_H
#define WIFI_OFF_H

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

/* Descriptor da imagem — definido em wifi_off.c */
extern const lv_img_dsc_t wifi_off;

#endif /* WIFI_OFF_H */