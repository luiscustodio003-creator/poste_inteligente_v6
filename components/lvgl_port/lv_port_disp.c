/* ============================================================
   LVGL DISPLAY PORT
   ============================================================ */

#include "lv_port_disp.h"
#include "st7789.h"
#include "lvgl.h"
#include "hw_config.h"

static lv_disp_draw_buf_t draw_buf;

static lv_color_t buf[LCD_H_RES * 20];

static void disp_flush(
    lv_disp_drv_t *drv,
    const lv_area_t *area,
    lv_color_t *color_p
)
{
    int w = area->x2 - area->x1 + 1;
    int h = area->y2 - area->y1 + 1;

    st7789_draw_bitmap(
        area->x1,
        area->y1,
        w,
        h,
        (uint16_t*)color_p
    );

    lv_disp_flush_ready(drv);
}

void lvgl_port_init(void)
{
    lv_init();

    st7789_init();

    lv_disp_draw_buf_init(
        &draw_buf,
        buf,
        NULL,
        LCD_H_RES * 20
    );

    static lv_disp_drv_t disp_drv;

    lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;

    disp_drv.flush_cb = disp_flush;

    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
}