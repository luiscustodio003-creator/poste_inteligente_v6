/* ============================================================
   LVGL CONFIGURAÇÃO
   ------------------------------------------------------------
   @file      lv_conf.h
   @brief     Configuração para display_manager v5.0
              ST7789 240×240 + lv_canvas radar HLK-LD2450
   
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno

   
============================================================ */

#if 1   /* Activa o conteúdo deste ficheiro — DEVE ser 1 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* ============================================================
   SUPRESSÃO DE CONFLITOS (Kconfig vs Manual)
   Limpa definições anteriores do lv_conf_internal.h para
   evitar warnings "redefined" durante a compilação.
   IMPORTANTE: a lista deve cobrir TODAS as defines abaixo.
============================================================ */
#undef LV_MEM_SIZE
#undef LV_MEM_BUF_MAX_NUM
#undef LV_DISP_DEF_REFR_PERIOD
#undef LV_INDEV_DEF_READ_PERIOD
#undef LV_DPI_DEF
#undef LV_DRAW_COMPLEX
#undef LV_SHADOW_CACHE_SIZE
#undef LV_CIRCLE_CACHE_SIZE
#undef LV_LAYER_SIMPLE_BUF_SIZE
#undef LV_COLOR_DEPTH
#undef LV_COLOR_16_SWAP          /* ← obrigatório — corrige cor ST7789 */

/* Fontes — lista completa para evitar warnings */
#undef LV_FONT_MONTSERRAT_8      /* ← adicionado v5.0 */
#undef LV_FONT_MONTSERRAT_10     /* ← adicionado v5.0 (usado no display) */
#undef LV_FONT_MONTSERRAT_12
#undef LV_FONT_MONTSERRAT_14
#undef LV_FONT_MONTSERRAT_16
#undef LV_FONT_MONTSERRAT_18     /* ← adicionado v5.0 (usado no display) */
#undef LV_FONT_MONTSERRAT_20
#undef LV_FONT_MONTSERRAT_24
#undef LV_FONT_DEFAULT

/* Widgets */
#undef LV_USE_ARC
#undef LV_USE_BAR
#undef LV_USE_BTN
#undef LV_USE_IMG
#undef LV_USE_LABEL
#undef LV_USE_LINE
#undef LV_USE_BTNMATRIX
#undef LV_USE_TEXTAREA
#undef LV_USE_SLIDER
#undef LV_USE_CANVAS
#undef LV_USE_CHART              /* ← adicionado v5.0 (desactivado) */
#undef LV_USE_CHECKBOX
#undef LV_USE_DROPDOWN
#undef LV_USE_ROLLER
#undef LV_USE_SWITCH

/* Temas */
#undef LV_USE_THEME_DEFAULT
#undef LV_THEME_DEFAULT_GROW
#undef LV_THEME_DEFAULT_TRANSITION_TIME
#undef LV_USE_THEME_BASIC


/* ============================================================
   MEMÓRIA E HAL
============================================================ */
#define LV_MEM_CUSTOM           0
#if LV_MEM_CUSTOM == 0
    /* 64 KB — necessário para heap LVGL com lv_canvas activo.
       O buffer do canvas (230×90×2 = ~41 KB) é estático e não
       conta para este heap, mas os objectos LVGL (labels, cards,
       bar, canvas obj) precisam de ~20-25 KB adicionais.        */
    #define LV_MEM_SIZE         (64U * 1024U)
    #define LV_MEM_ADR          0
#endif
#define LV_MEM_BUF_MAX_NUM      16
#define LV_MEMCPY_MEMSET_STD    0

#define LV_DISP_DEF_REFR_PERIOD     30   /* ms */
#define LV_INDEV_DEF_READ_PERIOD    30   /* ms */
#define LV_TICK_CUSTOM              0
#define LV_DPI_DEF                 130


/* ============================================================
   DESENHO
============================================================ */
#define LV_DRAW_COMPLEX             1
#define LV_SHADOW_CACHE_SIZE        0
#define LV_CIRCLE_CACHE_SIZE        4
#define LV_LAYER_SIMPLE_BUF_SIZE    (24 * 1024)


/* ============================================================
   COR — ST7789 240×240
   LV_COLOR_16_SWAP = 1 OBRIGATÓRIO para o ST7789.
   Com 0 os bytes RGB565 ficam trocados e todas as cores
   aparecem erradas no ecrã físico (vermelho→azul, etc).
============================================================ */
#define LV_COLOR_DEPTH          16
#define LV_COLOR_16_SWAP         1   /* ← CORRIGIDO v5.0 (era 0) */
#define LV_COLOR_SCREEN_TRANSP   0


/* ============================================================
   FONTES MONTSERRAT
   Activas apenas as usadas pelo display_manager v5.0:
     10  — subtítulo zona identidade, títulos cards, HLK tag
     12  — labels hardware, vizinhos
     14  — nome do poste, badge FSM
     18  — valores numéricos nos cards (T, Tc, km/h)
============================================================ */
#define LV_FONT_MONTSERRAT_8     0
#define LV_FONT_MONTSERRAT_10    1   /* subtítulo, títulos cards  */
#define LV_FONT_MONTSERRAT_12    1   /* labels hardware/vizinhos  */
#define LV_FONT_MONTSERRAT_14    1   /* nome poste, badge FSM     */
#define LV_FONT_MONTSERRAT_16    0   /* não usado no display v5.0 */
#define LV_FONT_MONTSERRAT_18    1   /* valores T, Tc, km/h       */
#define LV_FONT_MONTSERRAT_20    0   /* não usado no display v5.0 */
#define LV_FONT_MONTSERRAT_24    0

#define LV_FONT_DEFAULT    &lv_font_montserrat_14


/* ============================================================
   WIDGETS
   Activados apenas os necessários — reduz tamanho do binário.
============================================================ */

/* Usados pelo display_manager v5.0 */
#define LV_USE_LABEL        1   /* todos os textos               */
#define LV_USE_BAR          1   /* barra de brilho DALI          */
#define LV_USE_CANVAS       1   /* radar HLK-LD2450 ← CRÍTICO    */

/* Usados por outros módulos / UI futura */
#define LV_USE_ARC          1
#define LV_USE_BTN          1
#define LV_USE_IMG          1
#define LV_USE_LINE         1
#define LV_USE_BTNMATRIX    1
#define LV_USE_TEXTAREA     1
#define LV_USE_SLIDER       1

/* Desactivados — não usados no display v5.0 */
#define LV_USE_CHART        0   /* ← removido na v5.0 (era lv_chart) */
#define LV_USE_CHECKBOX     0
#define LV_USE_DROPDOWN     0
#define LV_USE_ROLLER       0
#define LV_USE_SWITCH       0


/* ============================================================
   TEMAS
============================================================ */
#define LV_USE_THEME_DEFAULT            1
    #define LV_THEME_DEFAULT_DARK               0
    #define LV_THEME_DEFAULT_GROW               1
    #define LV_THEME_DEFAULT_TRANSITION_TIME    80
#define LV_USE_THEME_BASIC              1


#endif /* LV_CONF_H */
#endif /* fim do #if 1 */