/* ============================================================
   DISPLAY MANAGER -- IMPLEMENTACAO
   ------------------------------------------------------------
   @file      display_manager.c
   @brief     Gestao do display ST7789 com LVGL 8.3
   @version   5.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alteracoes (v4.0 -> v5.0):
   --------------------------
   1. Adicionado label_luz -- mostra brilho actual (%) e estado
      FSM na mesma linha: "LUZ: 50%  [SAFE MODE]"
      Cor dinamica: verde=100%, ambar=11-99%, cinza<=10%.
      Separador adicional antes desta linha (y=168).
   2. Adicionado label_estado -- estado FSM em texto curto ao
      lado do brilho, entre parenteses rectos.
   3. label_vel movido para y=210 para acomodar nova linha.

   Layout do display 240x240:
   --------------------------
   [icone wifi]  [POSTE 01]     <- y=5 / y=10
   IP: 192.168.4.1              <- y=35
   VIZINHOS: 2                  <- y=55
   ─────────────────────────    <- separador y=73
   P02 .4.2 OK   52km  2s       <- y=80
   P03 .4.3 OFF  --km 15s       <- y=102
   P04 .4.4 SAFE 30km  8s       <- y=124
   P05 .4.5 OK   45km  1s       <- y=146
   ─────────────────────────    <- separador y=168
   LUZ: 50%  [SAFE MODE]        <- y=185  (NOVO)
   VEL: 52 km/h                 <- y=210

   Dependencias:
   -------------
   - display_manager.h
   - st7789.h, lvgl.h
   - wifi_off.h, wifi_on.h
   - udp_manager.h (neighbor_t)
   - FreeRTOS, esp_timer, lv_conf.h
============================================================ */

#include "display_manager.h"
#include "st7789.h"
#include "lvgl.h"
#include "wifi_off.h"
#include "wifi_on.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* -----------------------------------------------------------
   Buffers de desenho LVGL -- 240x40x2 bytes = 19200 bytes
   ----------------------------------------------------------- */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t         buf1[240 * 40];

/* -----------------------------------------------------------
   Handles dos widgets
   ----------------------------------------------------------- */
static lv_obj_t *label_poste;
static lv_obj_t *label_ip;
static lv_obj_t *label_vizinhos;
static lv_obj_t *label_vel;
static lv_obj_t *label_luz;       /* NOVO: brilho + estado FSM */
static lv_obj_t *wifi_icon;
static lv_obj_t *neighbor_rows[MAX_NEIGHBORS];

/* -----------------------------------------------------------
   flush_cb -- Envia regiao do buffer LVGL -> ST7789 via SPI
   ----------------------------------------------------------- */
static void flush_cb(lv_disp_drv_t *drv,
                     const lv_area_t *area,
                     lv_color_t *color_p)
{
    int w = area->x2 - area->x1 + 1;
    int h = area->y2 - area->y1 + 1;
    st7789_draw_bitmap(area->x1, area->y1, w, h, (uint16_t *)color_p);
    lv_disp_flush_ready(drv);
}

/* -----------------------------------------------------------
   wifi_icon_create -- Estado inicial: wifi_off, vermelho
   ALPHA_1BIT: recolor_opa obrigatorio para mostrar cor
   ----------------------------------------------------------- */
static void wifi_icon_create(lv_obj_t *parent)
{
    wifi_icon = lv_img_create(parent);
    lv_img_set_src(wifi_icon, &wifi_off);
    lv_obj_align(wifi_icon, LV_ALIGN_TOP_LEFT, 5, 5);

    lv_obj_set_style_img_opa(wifi_icon, LV_OPA_COVER,
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_img_recolor(wifi_icon, lv_color_make(255, 0, 0),
                                  LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_img_recolor_opa(wifi_icon, LV_OPA_COVER,
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
}

/* -----------------------------------------------------------
   display_manager_set_wifi -- Troca icone e cor Wi-Fi
   ----------------------------------------------------------- */
void display_manager_set_wifi(bool connected)
{
    if (!wifi_icon) return;

    if (connected)
    {
        lv_img_set_src(wifi_icon, &wifi_on);
        lv_obj_set_style_img_recolor(wifi_icon, lv_color_make(0, 255, 0),
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
        lv_img_set_src(wifi_icon, &wifi_off);
        lv_obj_set_style_img_recolor(wifi_icon, lv_color_make(255, 0, 0),
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    lv_obj_set_style_img_recolor_opa(wifi_icon, LV_OPA_COVER,
                                      LV_PART_MAIN | LV_STATE_DEFAULT);
}

/* -----------------------------------------------------------
   display_manager_set_ap_mode -- Prefixo IP/AP no label
   ----------------------------------------------------------- */
void display_manager_set_ap_mode(bool ap_active, const char *ip)
{
    if (!label_ip || !ip) return;
    static char buf[40];
    snprintf(buf, sizeof(buf), ap_active ? "AP: %s" : "IP: %s", ip);
    lv_label_set_text(label_ip, buf);
}

/* -----------------------------------------------------------
   display_manager_update_neighbors -- Tabela de vizinhos

   Para cada vizinho: "P<id> <ip_curto> <estado> <vel>km <t>s"
   Cor: verde=OK, vermelho=OFF, ambar=SAFE_MODE
   ----------------------------------------------------------- */
void display_manager_update_neighbors(const neighbor_t *neighbors,
                                      size_t            count)
{
    uint64_t now = (uint64_t)(esp_timer_get_time() / 1000ULL);

    for (size_t i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!neighbor_rows[i]) continue;

        if (i >= count || !neighbors[i].active)
        {
            lv_label_set_text(neighbor_rows[i], "");
            continue;
        }

        const neighbor_t *n = &neighbors[i];

        uint64_t elapsed_s = (n->last_seen > 0)
                             ? (now - n->last_seen) / 1000ULL
                             : 0;

        const char *estado;
        lv_color_t  cor;

        switch (n->status)
        {
            case NEIGHBOR_OK:
                estado = "OK  ";
                cor    = lv_color_make(0, 200, 0);
                break;
            case NEIGHBOR_SAFE_MODE:
                estado = "SAFE";
                cor    = lv_color_make(255, 180, 0);
                break;
            default:
                estado = "OFF ";
                cor    = lv_color_make(220, 0, 0);
                break;
        }

        static char buf[48];
        snprintf(buf, sizeof(buf),
                 "P%02d %.3s %s %3.0fkm %llus",
                 n->id,
                 n->ip + 10,
                 estado,
                 n->last_speed,
                 (unsigned long long)elapsed_s);

        lv_label_set_text(neighbor_rows[i], buf);
        lv_obj_set_style_text_color(neighbor_rows[i], cor,
                                     LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}

/* -----------------------------------------------------------
   display_manager_update_brightness -- Brilho + estado FSM

   Mostra na linha "LUZ: XX%  [ESTADO]" com cor dinamica:
     verde  = brilho 100% (LIGHT_ON activo)
     ambar  = brilho intermedio (DETECTION ou SAFE_MODE)
     cinza  = brilho minimo (IDLE ou TIMEOUT)

   Exemplos de saida:
     "LUZ: 100%  [LIGHT ON]"   -- veiculo presente
     "LUZ:  50%  [SAFE MODE]"  -- radar/wifi falharam
     "LUZ:  55%  [DETECTION]"  -- a confirmar veiculo
     "LUZ:  10%  [IDLE]"       -- em repouso
   ----------------------------------------------------------- */
void display_manager_update_brightness(uint8_t     brightness,
                                       const char *state_name)
{
    if (!label_luz) return;

    static char buf[40];
    snprintf(buf, sizeof(buf), "LUZ: %3d%%  [%s]",
             brightness, state_name ? state_name : "---");

    lv_label_set_text(label_luz, buf);

    /* Cor dinamica conforme nivel de brilho */
    lv_color_t cor;
    if (brightness >= 100)
        cor = lv_color_make(0, 220, 0);       /* verde -- brilho maximo */
    else if (brightness > 10)
        cor = lv_color_make(255, 180, 0);     /* ambar -- brilho intermedio */
    else
        cor = lv_color_make(160, 160, 160);   /* cinza -- brilho minimo */

    lv_obj_set_style_text_color(label_luz, cor,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
}

/* -----------------------------------------------------------
   display_manager_init -- Inicializa display, LVGL e widgets
   ----------------------------------------------------------- */
void display_manager_init(void)
{
    /* 1. Driver ST7789 */
    st7789_init();

    /* 2. Framework LVGL */
    lv_init();

    /* 3. Buffer de desenho */
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 240 * 40);

    /* 4. Regista driver */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = 240;
    disp_drv.ver_res  = 240;
    disp_drv.flush_cb = flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* 5. Desactiva tema */
    lv_disp_t *disp = lv_disp_get_default();
    lv_disp_set_theme(disp, NULL);

    lv_obj_t *scr = lv_scr_act();

    /* --- Fundo preto --- */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000),
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER,
                             LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(scr, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_color_t       amber = lv_color_hex(0xFFBF00);
    const lv_font_t *font  = &lv_font_montserrat_14;

    /* --- Icone Wi-Fi --- */
    wifi_icon_create(scr);

    /* --- Nome do poste --- */
    label_poste = lv_label_create(scr);
    lv_label_set_text(label_poste, "POSTE 01");
    lv_obj_set_style_text_color(label_poste, amber,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_poste, font,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_poste, LV_ALIGN_TOP_MID, 0, 10);

    /* --- IP / AP --- */
    label_ip = lv_label_create(scr);
    lv_label_set_text(label_ip, "IP: 0.0.0.0");
    lv_obj_set_style_text_color(label_ip, amber,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_ip, font,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_ip, LV_ALIGN_TOP_LEFT, 10, 35);

    /* --- Contagem de vizinhos --- */
    label_vizinhos = lv_label_create(scr);
    lv_label_set_text(label_vizinhos, "VIZINHOS: 0");
    lv_obj_set_style_text_color(label_vizinhos, amber,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_vizinhos, font,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_vizinhos, LV_ALIGN_TOP_LEFT, 10, 55);

    /* --- Separador superior (apos cabecalho) --- */
    lv_obj_t *sep1 = lv_obj_create(scr);
    lv_obj_set_size(sep1, 220, 1);
    lv_obj_set_style_bg_color(sep1, amber, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(sep1, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(sep1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(sep1, LV_ALIGN_TOP_MID, 0, 73);

    /* --- Tabela de vizinhos (ate MAX_NEIGHBORS linhas) --- */
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        neighbor_rows[i] = lv_label_create(scr);
        lv_label_set_text(neighbor_rows[i], "");
        lv_obj_set_style_text_font(neighbor_rows[i], font,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(neighbor_rows[i],
                                     lv_color_make(0, 200, 0),
                                     LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(neighbor_rows[i], LV_ALIGN_TOP_LEFT,
                     10, 80 + i * 22);
    }

    /* --- Separador inferior (antes de brilho/velocidade) --- */
    lv_obj_t *sep2 = lv_obj_create(scr);
    lv_obj_set_size(sep2, 220, 1);
    lv_obj_set_style_bg_color(sep2, amber, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(sep2, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(sep2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(sep2, LV_ALIGN_TOP_MID, 0, 168);

    /* --- Brilho + estado FSM (NOVO) --- */
    label_luz = lv_label_create(scr);
    lv_label_set_text(label_luz, "LUZ:  10%  [IDLE]");
    lv_obj_set_style_text_color(label_luz,
                                 lv_color_make(160, 160, 160),
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_luz, font,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_luz, LV_ALIGN_TOP_LEFT, 10, 185);

    /* --- Velocidade radar --- */
    label_vel = lv_label_create(scr);
    lv_label_set_text(label_vel, "VEL:  0 km/h");
    lv_obj_set_style_text_color(label_vel, amber,
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label_vel, font,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(label_vel, LV_ALIGN_TOP_LEFT, 10, 210);
}

/* -----------------------------------------------------------
   display_manager_update_info -- Actualiza labels principais
   Nota: ip ignorado quando vazio -- gerido por set_ap_mode()
   ----------------------------------------------------------- */
void display_manager_update_info(const char *poste,
                                 const char *ip,
                                 int         vizinhos,
                                 int         vel)
{
    if (label_poste) lv_label_set_text(label_poste, poste);

    /* Actualiza IP apenas se string nao vazia */
    if (label_ip && ip && ip[0] != '\0')
        lv_label_set_text(label_ip, ip);

    static char buf[32];

    if (label_vizinhos)
    {
        snprintf(buf, sizeof(buf), "VIZINHOS: %d", vizinhos);
        lv_label_set_text(label_vizinhos, buf);
    }
    if (label_vel)
    {
        snprintf(buf, sizeof(buf), "VEL: %3d km/h", vel);
        lv_label_set_text(label_vel, buf);
    }
}

/* -----------------------------------------------------------
   lvgl_task -- Loop LVGL a cada 10 ms
   ----------------------------------------------------------- */
static void lvgl_task(void *pv)
{
    while (1)
    {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* -----------------------------------------------------------
   display_manager_start -- Ponto de entrada
   ----------------------------------------------------------- */
void display_manager_start(void)
{
    display_manager_init();
    xTaskCreate(lvgl_task, "lvgl_task", 4096, NULL, 5, NULL);
}