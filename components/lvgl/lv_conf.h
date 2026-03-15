/* ============================================================
   LVGL CONFIGURAÇÃO
   ------------------------------------------------------------
   @file      lv_conf.h
   @brief     Configuração do LVGL v8.3.11 para display 240x240
   @version   2.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Correcções aplicadas (v1.0 → v2.0):
   -------------------------------------
   1. LV_USE_FREETYPE desactivado — não existe em LVGL 8.3 embutido
      no ESP-IDF; causa erro de compilação e impede o arranque.
   2. LV_USE_FONT_MANAGER desactivado — dependente do FreeType,
      inacessível na configuração standard do ESP-IDF.
   3. LV_FONT_MONTSERRAT_22 removido — não existe em LVGL 8.3,
      causava símbolos de fonte inválidos em tempo de execução.
   4. LV_FONT_DEFAULT mantido como montserrat_14 para compatibilidade
      com display_manager.c que referencia explicitamente essa fonte.
   5. LV_USE_THEME_DEFAULT mantido activo (necessário para lv_disp_set_theme).

   Dependências:
   -------------
   - LVGL 8.3.11
   - ESP-IDF (sem componente FreeType externo)
============================================================ */

#if 1   /* Activa o conteúdo deste ficheiro */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* ===========================================================
   MEMÓRIA
   =========================================================== */
#define LV_MEM_CUSTOM 0
#if LV_MEM_CUSTOM == 0
    /* 48 KB de heap interno para LVGL — adequado para 240x240 */
    #define LV_MEM_SIZE         (48U * 1024U)
    #define LV_MEM_ADR          0               /* Endereço automático */
#else
    #define LV_MEM_CUSTOM_INCLUDE   <stdlib.h>
    #define LV_MEM_CUSTOM_ALLOC     malloc
    #define LV_MEM_CUSTOM_FREE      free
    #define LV_MEM_CUSTOM_REALLOC   realloc
#endif
#define LV_MEM_BUF_MAX_NUM      16
#define LV_MEMCPY_MEMSET_STD    0

/* ===========================================================
   HAL — Hardware Abstraction Layer
   =========================================================== */
#define LV_DISP_DEF_REFR_PERIOD     30  /* ms — período de refrescamento */
#define LV_INDEV_DEF_READ_PERIOD    30  /* ms — período de leitura de input */
#define LV_TICK_CUSTOM              0   /* Usa lv_tick_inc() manualmente */
#define LV_DPI_DEF                  130 /* DPI adequado para 240x240 */

/* ===========================================================
   DESENHO
   =========================================================== */
#define LV_DRAW_COMPLEX                     1
#define LV_SHADOW_CACHE_SIZE                0
#define LV_CIRCLE_CACHE_SIZE                4
#define LV_LAYER_SIMPLE_BUF_SIZE            (24 * 1024)
#define LV_LAYER_SIMPLE_FALLBACK_BUF_SIZE   (3  * 1024)
#define LV_IMG_CACHE_DEF_SIZE               0
#define LV_GRADIENT_MAX_STOPS               2
#define LV_GRAD_CACHE_DEF_SIZE              0
#define LV_DITHER_GRADIENT                  0
#define LV_DISP_ROT_MAX_BUF                 (10 * 1024)

/* ===========================================================
   FONTES
   -------------------------------------------------------
   ATENÇÃO: Apenas activar fontes que realmente existam em
   LVGL 8.3 embutido. Fontes inexistentes causam erro de
   ligação silencioso — a fonte por defeito é usada sem aviso.

   Fontes disponíveis em LVGL 8.3 (lista parcial):
     8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 36,
     40, 44, 48
   =========================================================== */
#define LV_FONT_MONTSERRAT_8    0
#define LV_FONT_MONTSERRAT_10   0
#define LV_FONT_MONTSERRAT_12   0
#define LV_FONT_MONTSERRAT_14   1   /* Usada em display_manager.c */
#define LV_FONT_MONTSERRAT_16   1   /* Útil para valores/velocidade */
#define LV_FONT_MONTSERRAT_20   1   /* Útil para título do poste */
#define LV_FONT_MONTSERRAT_24   1   /* Tamanho máximo activado */
#define LV_FONT_MONTSERRAT_28   0
#define LV_FONT_MONTSERRAT_32   0
#define LV_FONT_MONTSERRAT_48   0

/* CORRECÇÃO: LV_FONT_MONTSERRAT_22 não existe em LVGL 8.3 — removido */

/* Fonte por defeito: montserrat_14 (referenciada em display_manager.c) */
#define LV_FONT_DEFAULT         &lv_font_montserrat_14

/* Funcionalidades de fonte desactivadas (sem suporte nativo no ESP-IDF) */
#define LV_USE_FONT_COMPRESSED  0
#define LV_FONT_FMT_TXT_LARGE   0
#define LV_USE_FONT_SUBPX       0
#define LV_USE_FONT_PLACEHOLDER 1

/* CORRECÇÃO: FreeType e Font Manager removidos — não existem no LVGL
   8.3 padrão do ESP-IDF. Activá-los causava falha de compilação. */
#define LV_USE_FONT_MANAGER     0
#define LV_USE_FREETYPE         0

/* ===========================================================
   WIDGETS
   =========================================================== */
#define LV_USE_ARC          1
#define LV_USE_BAR          1
#define LV_USE_BTN          1
#define LV_USE_BTNMATRIX    1
#define LV_USE_CANVAS       1
#define LV_USE_CHECKBOX     1
#define LV_USE_DROPDOWN     1
#define LV_USE_IMG          1   /* Necessário para o ícone Wi-Fi */
#define LV_USE_LABEL        1   /* Necessário para os labels do display */
    #define LV_LABEL_TEXT_SELECTION  1
    #define LV_LABEL_LONG_TXT_HINT   1
#define LV_USE_LINE         1
#define LV_USE_ROLLER       1
    #define LV_ROLLER_INF_PAGES      7
#define LV_USE_SLIDER       1
#define LV_USE_SWITCH       1
#define LV_USE_TEXTAREA     1
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500
#define LV_USE_TABLE        1

/* ===========================================================
   TEMAS
   -------------------------------------------------------
   ATENÇÃO: LV_USE_THEME_DEFAULT deve estar a 1 mesmo que
   display_manager.c chame lv_disp_set_theme(disp, NULL),
   pois o tema é inicializado internamente pelo LVGL.
   =========================================================== */
#define LV_USE_THEME_DEFAULT    1
    #define LV_THEME_DEFAULT_DARK               0
    #define LV_THEME_DEFAULT_GROW               1
    #define LV_THEME_DEFAULT_TRANSITION_TIME    80
#define LV_USE_THEME_BASIC      1
#define LV_USE_THEME_MONO       1

#endif /* LV_CONF_H */
#endif /* Fim do #if 1 */