/* ============================================================
   DISPLAY MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      display_manager.c
   @brief     Camada de apresentação LVGL + ST7789 — ecrã 240x240
   @version   3.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Divide o ecrã 240×240 em quatro zonas horizontais fixas.
   Cada zona é actualizada de forma independente pelas funções
   display_manager_set_*() chamadas pelos módulos de nível
   superior (state_machine, wifi_manager, comm_manager).

   Layout do ecrã (alturas em pixels):
   -------------------------------------
     y=  0..34  → ZONA IDENTIDADE  (nome + badge de modo)
     y= 35..39  → separador
     y= 40..89  → ZONA HARDWARE    (WiFi · Radar · DALI %)
     y= 90..94  → separador
     y= 95..154 → ZONA TRÁFEGO     (T | Tc | Vel km/h)
     y=155..159 → separador
     y=160..239 → ZONA VIZINHOS    (Esq / Dir)

   Alterações v2.2 → v3.0:
   ------------------------
   1. Adicionada ZONA HARDWARE: radar_ok + brilho DALI (%)
   2. Adicionada ZONA TRÁFEGO: contadores T e Tc + velocidade
   3. display_manager_set_neighbors() expandido com estado OK/OFF
   4. display_manager_set_status() implementado com cores por estado
   5. display_manager_set_leader() implementado (badge MASTER)
   6. display_manager_set_speed() implementado
   7. Separadores visuais entre zonas
   8. Barra de brilho DALI animada (proporcional a 0–100%)

   Dependências:
   -------------
   - st7789.h        : driver SPI do display físico
   - system_config.h : LCD_H_RES, LCD_V_RES, POSTE_NAME
   - hw_config.h     : LCD_PIN_*
   - lvgl            : v8.3.x

============================================================ */

#include "display_manager.h"
#include "st7789.h"
#include "system_config.h"
#include "hw_config.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <lvgl.h>

/* Etiqueta de log deste módulo */
static const char *TAG = "DISP_MGR";

/* ============================================================
   CORES — definidas como hex RGB565 convertido para LVGL
   Usamos lv_color_hex() para compatibilidade com o modo de
   cor configurado em lv_conf.h (RGB565, swap activado).
============================================================ */
#define COR_BRANCO      0xFFFFFF
#define COR_PRETO       0x000000
#define COR_CINZENTO    0x888780   /* texto secundário          */
#define COR_VERDE       0x22C55E   /* online / OK / MASTER      */
#define COR_VERMELHO    0xEF4444   /* offline / erro / AUTÓNOMO */
#define COR_AMARELO     0xFACC15   /* LIGHT ON / DALI activo    */
#define COR_LARANJA     0xF97316   /* SAFE MODE                 */
#define COR_AZUL        0x60A5FA   /* vizinho esquerdo / info   */
#define COR_VIOLETA     0xA78BFA   /* velocidade                */
#define COR_CIANO       0x38BDF8   /* Tc (a caminho)            */
#define COR_SEPARADOR   0x333333   /* linha divisória           */
#define COR_FUNDO_CARD  0x111111   /* fundo dos cards de T/Tc   */

/* ============================================================
   ELEMENTOS VISUAIS — estado interno do módulo
   Todos os ponteiros são iniciados por ui_create().
============================================================ */

/* --- Zona identidade --- */
static lv_obj_t *label_nome;      /* Nome do poste (fixo)       */
static lv_obj_t *label_badge;     /* Badge de modo da FSM       */

/* --- Zona hardware --- */
static lv_obj_t *label_wifi;      /* "WiFi: ON · 192.168.x.x"  */
static lv_obj_t *label_radar;     /* "Radar: OK · UART"         */
static lv_obj_t *label_dali;      /* "DALI: 100%"               */
static lv_obj_t *bar_dali;        /* Barra proporcional 0–100%  */

/* --- Zona tráfego --- */
static lv_obj_t *label_T_val;     /* Valor numérico de T        */
static lv_obj_t *label_Tc_val;    /* Valor numérico de Tc       */
static lv_obj_t *label_vel_val;   /* Velocidade km/h            */

/* --- Zona vizinhos --- */
static lv_obj_t *label_neb_esq;   /* IP + estado vizinho esq    */
static lv_obj_t *label_neb_dir;   /* IP + estado vizinho dir    */

/* ============================================================
   st7789_flush_cb
   ------------------------------------------------------------
   Callback de flush exigido pelo LVGL. Transfere o buffer
   de rendering para o display ST7789 via SPI.
============================================================ */
static void st7789_flush_cb(lv_disp_drv_t  *disp_drv,
                             const lv_area_t *area,
                             lv_color_t      *color_p)
{
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;

    st7789_draw_bitmap(area->x1, area->y1, w, h,
                       (const uint16_t *)color_p);

    lv_disp_flush_ready(disp_drv);
}

/* ============================================================
   _separador
   ------------------------------------------------------------
   Desenha uma linha horizontal fina como separador de zona.
   y_pos: coordenada y do centro da linha.
============================================================ */
static void _separador(lv_obj_t *pai, int y_pos)
{
    lv_obj_t *linha = lv_obj_create(pai);
    lv_obj_set_size(linha, LCD_H_RES - 10, 1);
    lv_obj_set_pos(linha, 5, y_pos);
    lv_obj_set_style_bg_color(linha, lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_bg_opa(linha, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(linha, 0, 0);
    lv_obj_set_style_radius(linha, 0, 0);
    lv_obj_set_style_pad_all(linha, 0, 0);
}

/* ============================================================
   _label_novo
   ------------------------------------------------------------
   Cria um lv_label com posição, cor e texto iniciais.
   Função auxiliar para reduzir repetição em ui_create().
============================================================ */
static lv_obj_t *_label_novo(lv_obj_t   *pai,
                              int         x,
                              int         y,
                              uint32_t    cor,
                              const char *texto)
{
    lv_obj_t *l = lv_label_create(pai);
    lv_label_set_text(l, texto);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_text_color(l, lv_color_hex(cor), 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    return l;
}

/* ============================================================
   ui_create
   ------------------------------------------------------------
   Cria todos os elementos visuais do ecrã.
   Chamada uma única vez durante display_manager_init().

   Coordenadas Y (referência):
     0  – início da zona identidade
     36 – separador
     41 – início da zona hardware
     92 – separador
     97 – início da zona tráfego
     156– separador
     161– início da zona vizinhos
============================================================ */
static void ui_create(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Fundo preto total */
    lv_obj_set_style_bg_color(scr, lv_color_hex(COR_PRETO), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* ----------------------------------------------------------
       ZONA IDENTIDADE (y: 4..34)
       Nome centrado + badge de modo alinhado à direita
    ---------------------------------------------------------- */

    /* Nome do poste — fixo, centrado, fonte maior */
    label_nome = lv_label_create(scr);
    lv_label_set_text(label_nome, POSTE_NAME);
    lv_obj_set_style_text_color(label_nome, lv_color_hex(COR_BRANCO), 0);
    lv_obj_set_style_text_font(label_nome, &lv_font_montserrat_14, 0);
    lv_obj_align(label_nome, LV_ALIGN_TOP_LEFT, 8, 6);

    /* Badge de modo — alinhado à direita, cor dinâmica */
    label_badge = lv_label_create(scr);
    lv_label_set_text(label_badge, "IDLE");
    lv_obj_set_style_text_color(label_badge, lv_color_hex(COR_CINZENTO), 0);
    lv_obj_set_style_text_font(label_badge, &lv_font_montserrat_14, 0);
    lv_obj_align(label_badge, LV_ALIGN_TOP_RIGHT, -8, 8);

    /* Separador zona identidade / hardware */
    _separador(scr, 36);

    /* ----------------------------------------------------------
       ZONA HARDWARE (y: 41..91)
       Três linhas: WiFi · Radar · DALI
    ---------------------------------------------------------- */

    label_wifi  = _label_novo(scr, 8, 42, COR_VERMELHO, "WiFi: OFF");
    label_radar = _label_novo(scr, 8, 60, COR_CINZENTO,  "Radar: ---");
    label_dali  = _label_novo(scr, 8, 78, COR_CINZENTO,  "DALI:  0%");

    /* Barra de brilho DALI — ocupa a parte direita da linha DALI */
    bar_dali = lv_bar_create(scr);
    lv_obj_set_size(bar_dali, 80, 8);
    lv_obj_set_pos(bar_dali, 148, 82);
    lv_bar_set_range(bar_dali, 0, 100);
    lv_bar_set_value(bar_dali, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_dali,
                              lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_dali,
                              lv_color_hex(COR_AMARELO), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_dali, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_dali, 4, LV_PART_INDICATOR);

    /* Separador zona hardware / tráfego */
    _separador(scr, 93);

    /* ----------------------------------------------------------
       ZONA TRÁFEGO (y: 97..155)
       Três cards lado a lado: T | Tc | Vel
    ---------------------------------------------------------- */

    /* Largura e posição dos três cards */
    const int CW  = 68;   /* largura de cada card         */
    const int CH  = 52;   /* altura de cada card          */
    const int CY  = 97;   /* y de início dos cards        */
    const int GAP = 4;    /* espaço entre cards           */
    const int CX0 = 6;    /* x do primeiro card           */
    const int CX1 = CX0 + CW + GAP;
    const int CX2 = CX1 + CW + GAP;

    /* Função lambda-like via macro local para criar card */
    #define CARD(cx, cy, cw, ch) do {                                   \
        lv_obj_t *c = lv_obj_create(scr);                               \
        lv_obj_set_size(c, cw, ch);                                      \
        lv_obj_set_pos(c, cx, cy);                                       \
        lv_obj_set_style_bg_color(c, lv_color_hex(COR_FUNDO_CARD), 0);  \
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);                    \
        lv_obj_set_style_border_color(c,                                 \
            lv_color_hex(0x333333), 0);                                  \
        lv_obj_set_style_border_width(c, 1, 0);                         \
        lv_obj_set_style_radius(c, 6, 0);                               \
        lv_obj_set_style_pad_all(c, 0, 0);                              \
    } while(0)

    CARD(CX0, CY, CW, CH);
    CARD(CX1, CY, CW, CH);
    CARD(CX2, CY, CW, CH);
    #undef CARD

    /* Etiquetas de cabeçalho dos cards (texto fixo) */
    lv_obj_t *lT_hdr  = _label_novo(scr, CX0+4, CY+4,
                                    COR_CINZENTO, "T (aqui)");
    lv_obj_t *lTc_hdr = _label_novo(scr, CX1+4, CY+4,
                                    COR_CINZENTO, "Tc (vem)");
    lv_obj_t *lV_hdr  = _label_novo(scr, CX2+4, CY+4,
                                    COR_CINZENTO, "km/h");

    /* Reduz fonte dos cabeçalhos para caber */
    lv_obj_set_style_text_font(lT_hdr,  &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(lTc_hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(lV_hdr,  &lv_font_montserrat_14, 0);

    /* Valores numéricos grandes dos cards */
    label_T_val   = _label_novo(scr, CX0 + CW/2 - 8, CY+22,
                                COR_AMARELO, "0");
    label_Tc_val  = _label_novo(scr, CX1 + CW/2 - 8, CY+22,
                                COR_CIANO,   "0");
    label_vel_val = _label_novo(scr, CX2 + CW/2 - 8, CY+22,
                                COR_VIOLETA, "0");

    /* Fonte maior para os valores numéricos */
    lv_obj_set_style_text_font(label_T_val,
                               &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(label_Tc_val,
                               &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(label_vel_val,
                               &lv_font_montserrat_14, 0);

    /* Separador zona tráfego / vizinhos */
    _separador(scr, 156);

    /* ----------------------------------------------------------
       ZONA VIZINHOS (y: 161..235)
       Duas linhas: vizinho esquerdo e vizinho direito
    ---------------------------------------------------------- */

    lv_obj_t *lEsq_hdr = _label_novo(scr, 8, 162, COR_CINZENTO, "Esq:");
    lv_obj_t *lDir_hdr = _label_novo(scr, 8, 196, COR_CINZENTO, "Dir:");
    lv_obj_set_style_text_font(lEsq_hdr, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(lDir_hdr, &lv_font_montserrat_14, 0);

    /* IP + estado dos vizinhos — dinamicamente actualizados */
    label_neb_esq = _label_novo(scr, 8, 176, COR_CINZENTO, "--- · ---");
    label_neb_dir = _label_novo(scr, 8, 210, COR_CINZENTO, "--- · ---");
    lv_obj_set_style_text_font(label_neb_esq, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(label_neb_dir, &lv_font_montserrat_14, 0);

    ESP_LOGI(TAG, "Interface v3.0 criada (240x240)");
}

/* ============================================================
   display_manager_init
   ------------------------------------------------------------
   Inicializa o display ST7789, o LVGL e a interface gráfica.
   Deve ser chamada uma única vez em app_main().
============================================================ */
void display_manager_init(void)
{
    ESP_LOGI(TAG, "A inicializar display ST7789 e LVGL...");

    /* Inicializa hardware do display */
    st7789_init();

    /* Inicializa biblioteca LVGL */
    lv_init();

    /* Configura buffer de rendering (10 linhas de 240 px) */
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t         buf[LCD_H_RES * 10];
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_H_RES * 10);

    /* Regista driver de display no LVGL */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = LCD_H_RES;
    disp_drv.ver_res  = LCD_V_RES;
    disp_drv.flush_cb = st7789_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Cria todos os elementos da interface */
    ui_create();

    ESP_LOGI(TAG, "Display LVGL v3.0 pronto (240x240)");
}

/* ============================================================
   display_manager_tick
   ------------------------------------------------------------
   Notifica o LVGL do tempo decorrido.
   Chamar a cada 1 ms via timer de sistema.
============================================================ */
void display_manager_tick(uint32_t ms)
{
    lv_tick_inc(ms);
}

/* ============================================================
   display_manager_task
   ------------------------------------------------------------
   Processa os timers e eventos internos do LVGL.
   Chamar a cada 5–10 ms a partir da task principal.
============================================================ */
void display_manager_task(void)
{
    lv_timer_handler();
}

/* ============================================================
   display_manager_set_status
   ------------------------------------------------------------
   Actualiza o badge de modo com texto e cor adequados.

   Mapeamento de cores:
     "IDLE"      → cinzento  (sem tráfego)
     "LIGHT ON"  → amarelo   (luz a 100%)
     "SAFE MODE" → laranja   (radar falhou)
     "MASTER"    → verde     (líder da cadeia)
     "AUTÓNOMO"  → vermelho  (sem UDP)
============================================================ */
void display_manager_set_status(const char *status)
{
    if (!label_badge || !status) return;

    lv_label_set_text(label_badge, status);

    /* Selecciona cor conforme o estado */
    uint32_t cor = COR_CINZENTO; /* default: IDLE */

    if      (strcmp(status, "LIGHT ON")  == 0) cor = COR_AMARELO;
    else if (strcmp(status, "SAFE MODE") == 0) cor = COR_LARANJA;
    else if (strcmp(status, "MASTER")    == 0) cor = COR_VERDE;
    else if (strcmp(status, "AUTONOMO")  == 0) cor = COR_VERMELHO;

    lv_obj_set_style_text_color(label_badge,
                                lv_color_hex(cor), 0);
}

/* ============================================================
   display_manager_set_leader
   ------------------------------------------------------------
   Atalho que força o badge para "MASTER" verde ou "IDLE".
   Útil quando a FSM transita directamente sem passar por
   display_manager_set_status().
============================================================ */
void display_manager_set_leader(bool is_leader)
{
    display_manager_set_status(is_leader ? "MASTER" : "IDLE");
}

/* ============================================================
   display_manager_set_wifi
   ------------------------------------------------------------
   Actualiza o estado Wi-Fi e o IP na zona de hardware.
============================================================ */
void display_manager_set_wifi(bool connected, const char *ip)
{
    if (!label_wifi) return;

    if (connected)
    {
        /* Monta string "WiFi: ON · x.x.x.x" */
        char buf[48];
        snprintf(buf, sizeof(buf), "WiFi: ON  %s",
                 (ip && ip[0]) ? ip : "---");
        lv_label_set_text(label_wifi, buf);
        lv_obj_set_style_text_color(label_wifi,
                                    lv_color_hex(COR_VERDE), 0);
    }
    else
    {
        lv_label_set_text(label_wifi, "WiFi: OFF");
        lv_obj_set_style_text_color(label_wifi,
                                    lv_color_hex(COR_VERMELHO), 0);
    }
}

/* ============================================================
   display_manager_set_hardware
   ------------------------------------------------------------
   Actualiza estado do radar e brilho DALI na zona hardware.
   A barra proporcional reflecte o brilho visualmente.
============================================================ */
void display_manager_set_hardware(bool radar_ok, uint8_t brightness)
{
    /* --- Radar --- */
    if (label_radar)
    {
        if (radar_ok)
        {
            lv_label_set_text(label_radar, "Radar: OK");
            lv_obj_set_style_text_color(label_radar,
                                        lv_color_hex(COR_VERDE), 0);
        }
        else
        {
            lv_label_set_text(label_radar, "Radar: FALHA");
            lv_obj_set_style_text_color(label_radar,
                                        lv_color_hex(COR_VERMELHO), 0);
        }
    }

    /* --- DALI: texto percentagem --- */
    if (label_dali)
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "DALI: %3d%%", brightness);
        lv_label_set_text(label_dali, buf);

        /* Cor do texto conforme brilho: amarelo se > LIGHT_MIN */
        uint32_t cor = (brightness > LIGHT_MIN)
                       ? COR_AMARELO : COR_CINZENTO;
        lv_obj_set_style_text_color(label_dali,
                                    lv_color_hex(cor), 0);
    }

    /* --- Barra de brilho proporcional --- */
    if (bar_dali)
    {
        lv_bar_set_value(bar_dali, (int)brightness, LV_ANIM_ON);
    }
}

/* ============================================================
   display_manager_set_traffic
   ------------------------------------------------------------
   Actualiza os contadores T e Tc na zona de tráfego.
   T  = carros confirmados pelo radar local (amarelo)
   Tc = carros anunciados via UDP a caminho (ciano)
============================================================ */
void display_manager_set_traffic(int T, int Tc)
{
    if (label_T_val)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", T);
        lv_label_set_text(label_T_val, buf);

        /* Amarelo se T > 0, senão cinzento */
        uint32_t cor = (T > 0) ? COR_AMARELO : COR_CINZENTO;
        lv_obj_set_style_text_color(label_T_val,
                                    lv_color_hex(cor), 0);
    }

    if (label_Tc_val)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", Tc);
        lv_label_set_text(label_Tc_val, buf);

        /* Ciano se Tc > 0, senão cinzento */
        uint32_t cor = (Tc > 0) ? COR_CIANO : COR_CINZENTO;
        lv_obj_set_style_text_color(label_Tc_val,
                                    lv_color_hex(cor), 0);
    }
}

/* ============================================================
   display_manager_set_speed
   ------------------------------------------------------------
   Actualiza o campo de velocidade na zona de tráfego.
   0 km/h → cinzento; > 0 → violeta.
============================================================ */
void display_manager_set_speed(int speed)
{
    if (!label_vel_val) return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", speed);
    lv_label_set_text(label_vel_val, buf);

    uint32_t cor = (speed > 0) ? COR_VIOLETA : COR_CINZENTO;
    lv_obj_set_style_text_color(label_vel_val,
                                lv_color_hex(cor), 0);
}

/* ============================================================
   display_manager_set_neighbors
   ------------------------------------------------------------
   Actualiza os IPs e estados dos vizinhos esq e dir.
   Vizinho online → azul; offline → vermelho.
============================================================ */
void display_manager_set_neighbors(const char *nebL,
                                   const char *nebR,
                                   bool        leftOk,
                                   bool        rightOk)
{
    /* --- Vizinho esquerdo --- */
    if (label_neb_esq)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%s · %s",
                 (nebL && nebL[0]) ? nebL : "---",
                 leftOk ? "OK" : "OFF");
        lv_label_set_text(label_neb_esq, buf);
        lv_obj_set_style_text_color(label_neb_esq,
                                    lv_color_hex(leftOk
                                                 ? COR_AZUL
                                                 : COR_VERMELHO),
                                    0);
    }

    /* --- Vizinho direito --- */
    if (label_neb_dir)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%s · %s",
                 (nebR && nebR[0]) ? nebR : "---",
                 rightOk ? "OK" : "OFF");
        lv_label_set_text(label_neb_dir, buf);
        lv_obj_set_style_text_color(label_neb_dir,
                                    lv_color_hex(rightOk
                                                 ? COR_AZUL
                                                 : COR_VERMELHO),
                                    0);
    }
}