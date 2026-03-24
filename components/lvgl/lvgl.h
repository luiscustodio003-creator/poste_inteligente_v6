/**
 * @file lvgl.h
 * Include all LVGL related headers - v8.3.11
 */

#ifndef LVGL_H
#define LVGL_H

/* ===========================================================
    CONFIGURATION INCLUSION
    -----------------------------------------------------------
    Este bloco garante que o teu lv_conf.h é lido antes de 
    qualquer outro ficheiro da biblioteca.
   =========================================================== */
#if defined(LV_CONF_PATH)
#  define __LV_TO_STR_AUX(x) #x
#  define __LV_TO_STR(x) __LV_TO_STR_AUX(x)
#  include __LV_TO_STR(LV_CONF_PATH)
#  undef __LV_TO_STR_AUX
#  undef __LV_TO_STR
#elif defined(LV_CONF_INCLUDE_SIMPLE)
#  include "lv_conf.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* --- Versão da Biblioteca --- */
#define LVGL_VERSION_MAJOR 8
#define LVGL_VERSION_MINOR 3
#define LVGL_VERSION_PATCH 11
#define LVGL_VERSION_INFO ""

/* --- Includes de Infraestrutura (Misc) --- */
#include "src/misc/lv_log.h"
#include "src/misc/lv_timer.h"
#include "src/misc/lv_math.h"
#include "src/misc/lv_mem.h"
#include "src/misc/lv_async.h"
#include "src/misc/lv_anim_timeline.h"
#include "src/misc/lv_printf.h"

/* --- Includes de Abstração de Hardware (HAL) --- */
#include "src/hal/lv_hal.h"

/* --- Includes do Core --- */
#include "src/core/lv_obj.h"
#include "src/core/lv_group.h"
#include "src/core/lv_indev.h"
#include "src/core/lv_refr.h"
#include "src/core/lv_disp.h"
#include "src/core/lv_theme.h"

/* --- Includes de Fontes --- */
#include "src/font/lv_font.h"
#include "src/font/lv_font_loader.h"
#include "src/font/lv_font_fmt_txt.h"

/* --- Includes de Widgets (Objectos UI) --- */
#include "src/widgets/lv_arc.h"
#include "src/widgets/lv_btn.h"
#include "src/widgets/lv_img.h"
#include "src/widgets/lv_label.h"
#include "src/widgets/lv_line.h"
#include "src/widgets/lv_table.h"
#include "src/widgets/lv_checkbox.h"
#include "src/widgets/lv_bar.h"
#include "src/widgets/lv_slider.h"
#include "src/widgets/lv_btnmatrix.h"
#include "src/widgets/lv_dropdown.h"
#include "src/widgets/lv_roller.h"
#include "src/widgets/lv_textarea.h"
#include "src/widgets/lv_canvas.h"
#include "src/widgets/lv_switch.h"

/* --- Includes de Desenho e Extras --- */
#include "src/draw/lv_draw.h"
#include "src/lv_api_map.h"
#include "src/extra/lv_extra.h"

/* --- Helpers de Versão --- */
#define LV_VERSION_CHECK(x,y,z) (x == LVGL_VERSION_MAJOR && (y < LVGL_VERSION_MINOR || (y == LVGL_VERSION_MINOR && z <= LVGL_VERSION_PATCH)))

static inline int lv_version_major(void) { return LVGL_VERSION_MAJOR; }
static inline int lv_version_minor(void) { return LVGL_VERSION_MINOR; }
static inline int lv_version_patch(void) { return LVGL_VERSION_PATCH; }
static inline const char *lv_version_info(void) { return LVGL_VERSION_INFO; }

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LVGL_H */