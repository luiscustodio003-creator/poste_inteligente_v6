/* ============================================================
   DISPLAY MANAGER — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Gere o display TFT 240x240 com LVGL 8.3.
   Actualiza todos os widgets em tempo real a cada chamada.
   A lvgl_task corre a 10ms e trata o rendering.

   Ref LVGL 8.3: https://docs.lvgl.io/8.3/
   Ref ST7789:   https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_master/lcd
============================================================ */

#include "display_manager.h"
#include "st7789.h"
#include "lvgl.h"
#include "system_config.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "DISP_MGR";

/* -----------------------------------------------------------
   Buffer de desenho LVGL -- 240x40 pixeis = 19200 bytes
   ----------------------------------------------------------- */
static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t         s_buf[240 * 40];

/* -----------------------------------------------------------
   Handles dos widgets LVGL
   ----------------------------------------------------------- */
static lv_obj_t *lbl_nome;           /* Nome do poste + estado linha     */
static lv_obj_t *lbl_ip;             /* IP com prefixo AP/IP             */
static lv_obj_t *lbl_vizinhos_lr;    /* Vizinhos ESQ e DIR               */
static lv_obj_t *lbl_trafego;        /* Contadores T, Tc e velocidade    */
static lv_obj_t *lbl_rows[MAX_NEIGHBORS]; /* Tabela de vizinhos          */
static lv_obj_t *lbl_luz;            /* Brilho + estado FSM              */

/* Cores utilizadas */
#define COR_AMBER   lv_color_hex(0xFFBF00)
#define COR_VERDE   lv_color_hex(0x00C853)
#define COR_VERMELHO lv_color_hex(0xFF5252)
#define COR_CINZA   lv_color_hex(0x888888)
#define COR_AZUL    lv_color_hex(0x378ADD)
#define COR_FUNDO   lv_color_hex(0x000000)

/* ===========================================================
   FLUSH CALLBACK — Envia buffer para ST7789 via SPI
   =========================================================== */

static void flush_cb(lv_disp_drv_t   *drv,
                     const lv_area_t *area,
                     lv_color_t      *color_p)
{
    int w = area->x2 - area->x1 + 1;
    int h = area->y2 - area->y1 + 1;
    st7789_draw_bitmap(area->x1, area->y1, w, h, (uint16_t *)color_p);
    lv_disp_flush_ready(drv);
}

/* ===========================================================
   INICIALIZACAO DO DISPLAY E LVGL
   =========================================================== */

static void display_manager_init(void)
{
    /* 1. Inicializa driver ST7789 */
    st7789_init();

    /* 2. Inicializa LVGL */
    lv_init();

    /* 3. Buffer de desenho */
    lv_disp_draw_buf_init(&s_draw_buf, s_buf, NULL, 240 * 40);

    /* 4. Regista driver de display */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = 240;
    disp_drv.ver_res  = 240;
    disp_drv.flush_cb = flush_cb;
    disp_drv.draw_buf = &s_draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* 5. Sem tema -- controlo total das cores */
    lv_disp_t *disp = lv_disp_get_default();
    lv_disp_set_theme(disp, NULL);

    lv_obj_t *scr = lv_scr_act();

    /* Fundo preto */
    lv_obj_set_style_bg_color(scr,  COR_FUNDO, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr,    LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(scr, 0, LV_PART_MAIN);

    const lv_font_t *font = &lv_font_montserrat_14;

    /* --- Nome do poste + estado linha (y=8) --- */
    lbl_nome = lv_label_create(scr);
    lv_label_set_text(lbl_nome, "POSTE 01");
    lv_obj_set_style_text_color(lbl_nome, COR_AMBER, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_nome,  font,      LV_PART_MAIN);
    lv_obj_align(lbl_nome, LV_ALIGN_TOP_MID, 0, 8);

    /* --- IP com prefixo (y=26) --- */
    lbl_ip = lv_label_create(scr);
    lv_label_set_text(lbl_ip, "IP: 0.0.0.0");
    lv_obj_set_style_text_color(lbl_ip, COR_AMBER, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_ip,  font,      LV_PART_MAIN);
    lv_obj_align(lbl_ip, LV_ALIGN_TOP_LEFT, 8, 26);

    /* --- Separador 1 (y=40) --- */
    lv_obj_t *sep1 = lv_obj_create(scr);
    lv_obj_set_size(sep1, 224, 1);
    lv_obj_set_style_bg_color(sep1,    COR_AMBER,    LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sep1,      LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sep1, 0,           LV_PART_MAIN);
    lv_obj_align(sep1, LV_ALIGN_TOP_MID, 0, 40);

    /* --- Vizinhos ESQ e DIR (y=52) --- */
    lbl_vizinhos_lr = lv_label_create(scr);
    lv_label_set_text(lbl_vizinhos_lr, "ESQ: ---  |  DIR: ---");
    lv_obj_set_style_text_color(lbl_vizinhos_lr, COR_CINZA, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_vizinhos_lr,  font,      LV_PART_MAIN);
    lv_obj_align(lbl_vizinhos_lr, LV_ALIGN_TOP_LEFT, 8, 52);

    /* --- Separador 2 (y=65) --- */
    lv_obj_t *sep2 = lv_obj_create(scr);
    lv_obj_set_size(sep2, 224, 1);
    lv_obj_set_style_bg_color(sep2,    COR_AMBER,    LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sep2,      LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sep2, 0,           LV_PART_MAIN);
    lv_obj_align(sep2, LV_ALIGN_TOP_MID, 0, 65);

    /* --- Contadores T, Tc e velocidade (y=77) --- */
    lbl_trafego = lv_label_create(scr);
    lv_label_set_text(lbl_trafego, "T:0  Tc:0  VEL: -- km/h");
    lv_obj_set_style_text_color(lbl_trafego, COR_CINZA, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_trafego,  font,      LV_PART_MAIN);
    lv_obj_align(lbl_trafego, LV_ALIGN_TOP_LEFT, 8, 77);

    /* --- Separador 3 (y=90) --- */
    lv_obj_t *sep3 = lv_obj_create(scr);
    lv_obj_set_size(sep3, 224, 1);
    lv_obj_set_style_bg_color(sep3,    COR_AMBER,    LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sep3,      LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sep3, 0,           LV_PART_MAIN);
    lv_obj_align(sep3, LV_ALIGN_TOP_MID, 0, 90);

    /* --- Tabela de vizinhos (y=102 + i*20) --- */
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        lbl_rows[i] = lv_label_create(scr);
        lv_label_set_text(lbl_rows[i], "");
        lv_obj_set_style_text_font(lbl_rows[i],  font,     LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl_rows[i], COR_VERDE, LV_PART_MAIN);
        lv_obj_align(lbl_rows[i], LV_ALIGN_TOP_LEFT, 8, 102 + i * 20);
    }

    /* --- Separador 4 (y=178) --- */
    lv_obj_t *sep4 = lv_obj_create(scr);
    lv_obj_set_size(sep4, 224, 1);
    lv_obj_set_style_bg_color(sep4,    COR_AMBER,    LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sep4,      LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(sep4, 0,           LV_PART_MAIN);
    lv_obj_align(sep4, LV_ALIGN_TOP_MID, 0, 178);

    /* --- Brilho + estado FSM (y=192) --- */
    lbl_luz = lv_label_create(scr);
    lv_label_set_text(lbl_luz, "LUZ:  10%  [IDLE]");
    lv_obj_set_style_text_color(lbl_luz, COR_CINZA, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_luz,  font,      LV_PART_MAIN);
    lv_obj_align(lbl_luz, LV_ALIGN_TOP_LEFT, 8, 192);

    ESP_LOGI(TAG, "Display inicializado");
}

/* ===========================================================
   ACTUALIZACOES DOS WIDGETS
   =========================================================== */

/* Actualiza prefixo IP/AP no label */
void display_manager_set_ap_mode(bool ap_active, const char *ip)
{
    if (!lbl_ip || !ip) return;
    static char buf[40];
    snprintf(buf, sizeof(buf), ap_active ? "AP: %s" : "IP: %s", ip);
    lv_label_set_text(lbl_ip, buf);
}

/* Actualiza nome do poste e estado da linha */
void display_manager_update_info(const char *nome, const char *estado_linha)
{
    if (!lbl_nome) return;
    static char buf[48];
    if (estado_linha && estado_linha[0] != '\0')
        snprintf(buf, sizeof(buf), "%s  [%s]", nome, estado_linha);
    else
        snprintf(buf, sizeof(buf), "%s", nome);
    lv_label_set_text(lbl_nome, buf);

    /* Cor do nome conforme estado */
    lv_color_t cor = COR_AMBER;
    if (estado_linha)
    {
        if (strstr(estado_linha, "MASTER"))   cor = COR_VERDE;
        if (strstr(estado_linha, "AUTONOMO")) cor = COR_VERMELHO;
    }
    lv_obj_set_style_text_color(lbl_nome, cor, LV_PART_MAIN);
}

/* Actualiza linha ESQ/DIR com estado dos vizinhos */
void display_manager_update_neighbors_lr(const char *esq_str,
                                          const char *dir_str)
{
    if (!lbl_vizinhos_lr) return;
    static char buf[48];
    snprintf(buf, sizeof(buf), "ESQ:%s | DIR:%s",
             esq_str ? esq_str : "---",
             dir_str ? dir_str : "---");
    lv_label_set_text(lbl_vizinhos_lr, buf);

    /* Verde se ambos OK, vermelho se algum offline */
    bool ok = (!strstr(buf, "OFF") && !strstr(buf, "---"));
    lv_obj_set_style_text_color(lbl_vizinhos_lr,
                                 ok ? COR_VERDE : COR_AMBER,
                                 LV_PART_MAIN);
}

/* Actualiza contadores T, Tc e velocidade */
void display_manager_update_traffic(int T, int Tc, int vel_kmh)
{
    if (!lbl_trafego) return;
    static char buf[40];
    if (vel_kmh > 0)
        snprintf(buf, sizeof(buf), "T:%d  Tc:%d  VEL:%3d km/h", T, Tc, vel_kmh);
    else
        snprintf(buf, sizeof(buf), "T:%d  Tc:%d  VEL:  -- km/h", T, Tc);
    lv_label_set_text(lbl_trafego, buf);

    /* Cor: amarelo se ha trafego, cinza se vazio */
    lv_color_t cor = (T > 0 || Tc > 0) ? COR_AMBER : COR_CINZA;
    lv_obj_set_style_text_color(lbl_trafego, cor, LV_PART_MAIN);
}

/* Actualiza tabela completa de vizinhos */
void display_manager_update_neighbors(const neighbor_t *neighbors,
                                       size_t            count)
{
    if (!neighbors) return;

    uint64_t now = (uint64_t)(esp_timer_get_time() / 1000ULL);

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!lbl_rows[i]) continue;

        /* Limpa linha se sem vizinho */
        if ((size_t)i >= count || !neighbors[i].active)
        {
            lv_label_set_text(lbl_rows[i], "");
            continue;
        }

        const neighbor_t *n = &neighbors[i];
        uint64_t elapsed = (n->last_seen > 0) ?
                           (now - n->last_seen) / 1000ULL : 0;

        const char *estado;
        lv_color_t  cor;

        switch (n->status)
        {
            case NEIGHBOR_OK:
                estado = "OK  "; cor = COR_VERDE;    break;
            case NEIGHBOR_SAFE_MODE:
                estado = "SAFE"; cor = COR_AMBER;    break;
            case NEIGHBOR_MASTER:
                estado = "MST "; cor = COR_AZUL;     break;
            default:
                estado = "OFF "; cor = COR_VERMELHO; break;
        }

        static char buf[48];
        snprintf(buf, sizeof(buf),
                 "P%02d %.3s %s %3.0fkm %lus",
                 n->id,
                 n->ip + 10,   /* Mostra apenas os ultimos 3 chars do IP */
                 estado,
                 n->last_speed,
                 (unsigned long)elapsed);

        lv_label_set_text(lbl_rows[i], buf);
        lv_obj_set_style_text_color(lbl_rows[i], cor, LV_PART_MAIN);
    }
}

/* Actualiza brilho e estado FSM com cor dinamica */
void display_manager_update_brightness(uint8_t brightness,
                                        const char *state_name)
{
    if (!lbl_luz) return;

    static char buf[40];
    snprintf(buf, sizeof(buf), "LUZ: %3d%%  [%s]",
             brightness, state_name ? state_name : "---");
    lv_label_set_text(lbl_luz, buf);

    /* Cor conforme nivel de brilho */
    lv_color_t cor;
    if      (brightness >= 100) cor = COR_VERDE;    /* Verde -- 100% */
    else if (brightness >= 50)  cor = COR_AMBER;    /* Ambar -- 50%  */
    else                        cor = COR_CINZA;    /* Cinza -- 10%  */

    lv_obj_set_style_text_color(lbl_luz, cor, LV_PART_MAIN);
}

/* ===========================================================
   LVGL TASK — Loop de rendering a 10ms
   =========================================================== */

static void lvgl_task(void *pv)
{
    while (1)
    {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ===========================================================
   PONTO DE ENTRADA
   =========================================================== */

void display_manager_start(void)
{
    display_manager_init();
    xTaskCreate(lvgl_task, "lvgl_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Display pronto");
}