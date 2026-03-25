/* ============================================================
   DISPLAY MANAGER — IMPLEMENTAÇÃO v5.3
   ------------------------------------------------------------
   @file      display_manager.c
   @brief     Camada de apresentação LVGL + ST7789 — ecrã 240×240
   @version   5.3
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Descrição:
   ----------
   Layout final aprovado com radar em tempo real via lv_canvas
   e visualização HLK-LD2450 com sweep animado, halos e rastos.

   Layout do ecrã v5.3 (alturas em pixels):
   ------------------------------------------
     y=  0.. 35  → ZONA IDENTIDADE   (subtítulo + nome NVS + badge FSM)
     y= 36.. 36  → separador
     y= 37.. 92  → ZONA HARDWARE     (WiFi+Radar / Vizinhos / DALI+barra)
     y= 93.. 93  → separador
     y= 94..145  → ZONA TRÁFEGO      (cards T | Tc | km/h)
     y=146..146  → separador
     y=147..239  → ZONA RADAR        (lv_canvas 230×90 px, HLK-LD2450 X/Y)

   CORRECÇÃO CRÍTICA v5.2 → v5.3 — THREAD SAFETY (Task Watchdog):
   -----------------------------------------------------------------
   Problema: o LVGL não é thread-safe. As funções set_*() eram
   chamadas directamente por fsm_task e main_task, enquanto
   lv_timer_handler() corria também na main_task. Isto causava
   acesso concorrente ao heap do LVGL (lv_tlsf_malloc /
   lv_tlsf_free) e disparava o Task Watchdog por bloqueio de CPU.

   Backtrace típico do problema:
     lv_label_set_text → lv_mem_alloc → lv_tlsf_malloc (bloqueio)
     lv_label_set_text → lv_mem_free  → lv_tlsf_free   (bloqueio)

   Solução — padrão de fila de mensagens (Message Queue):
     1. Todas as funções set_*() são agora NON-BLOCKING:
        apenas empacotam os dados numa struct dm_msg_t
        e fazem xQueueSend() para s_fila (capacidade 16).
        Nunca tocam no LVGL directamente.

     2. display_manager_task() — chamada exclusivamente pela
        main_task — drena a fila com xQueueReceive(0) e aplica
        cada mensagem ao LVGL, depois chama lv_timer_handler().
        É a ÚNICA função que toca em objectos LVGL.

     3. display_manager_tick() continua a chamar lv_tick_inc()
        — esta é thread-safe por ser apenas um incremento atómico.

   Resultado: todo o LVGL corre numa única task (main_task),
   eliminando a concorrência e o watchdog.

   Alterações v5.0 → v5.2 (mantidas):
   -------------------------------------
   CORRECÇÃO 1: COR_VERDE corrigido para 0x22C55E
   CORRECÇÃO 2: COR_VERMELHO corrigido para 0xFF3333
   CORRECÇÃO 3: RADAR_XSPAN_MM definido localmente
   CORRECÇÃO 4: radar_obj_t vem de radar_manager.h
   MELHORIA 1-4: paleta, sweep, bordas, badge

   Dependências:
   -------------
   - display_manager.h : API pública (inclui radar_manager.h → radar_obj_t)
   - st7789.h          : driver SPI do display físico
   - system_config.h   : LCD_H_RES, LCD_V_RES, LIGHT_MIN, RADAR_MAX_MM
   - hw_config.h       : LCD_PIN_*
   - post_config.h     : post_get_name(), post_get_id()
   - lvgl              : v8.3.x (LV_USE_CANVAS=1 em lv_conf.h)

============================================================ */

#include "display_manager.h"
#include "st7789.h"
#include "system_config.h"
#include "hw_config.h"
#include "post_config.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ============================================================
   ETIQUETA DE LOG
============================================================ */
static const char *TAG = "DISP_MGR";

/* ============================================================
   FILA DE MENSAGENS — THREAD SAFETY
   ------------------------------------------------------------
   Todas as funções set_*() enviam mensagens para esta fila.
   display_manager_task() drena a fila e aplica ao LVGL.
   Capacidade: 16 mensagens — suficiente para um ciclo FSM
   (máximo ~8 chamadas set_* por iteração de 100ms).
============================================================ */
#define DM_FILA_CAP  16

/* Tipos de mensagem — identificam qual campo actualizar */
typedef enum {
    DM_MSG_STATUS = 0,
    DM_MSG_WIFI,
    DM_MSG_HARDWARE,
    DM_MSG_TRAFFIC,
    DM_MSG_SPEED,
    DM_MSG_NEIGHBORS,
    DM_MSG_RADAR,
} dm_msg_tipo_t;

/* Payload máximo por mensagem — union para não desperdiçar RAM */
typedef struct {
    dm_msg_tipo_t tipo;
    union {
        /* DM_MSG_STATUS */
        struct { char status[16]; } st;
        /* DM_MSG_WIFI */
        struct { bool connected; char ip[16]; } wifi;
        /* DM_MSG_HARDWARE */
        struct { bool radar_ok; uint8_t brightness; } hw;
        /* DM_MSG_TRAFFIC */
        struct { int T; int Tc; } traf;
        /* DM_MSG_SPEED */
        struct { int speed; } spd;
        /* DM_MSG_NEIGHBORS */
        struct {
            char nebL[16]; char nebR[16];
            bool leftOk;   bool rightOk;
        } neb;
        /* DM_MSG_RADAR */
        struct {
            radar_obj_t objs[RADAR_MAX_OBJ];
            uint8_t     count;
        } radar;
    };
} dm_msg_t;

static QueueHandle_t s_fila = NULL;

/* ============================================================
   CONSTANTE EM FALTA — RADAR_XSPAN_MM
   ------------------------------------------------------------
   Define o alcance lateral máximo do sensor (mm).
   Usado para escalar o eixo X do canvas radar.
   RADAR_MAX_MM está em system_config.h (= RADAR_MAX_M * 1000).
   O alcance lateral é igual ao alcance frontal (campo 180°).
============================================================ */
#ifndef RADAR_XSPAN_MM
#define RADAR_XSPAN_MM  RADAR_MAX_MM   /* alcance lateral = frontal */
#endif

/* ============================================================
   PALETA DE CORES — hex RGB888
   ------------------------------------------------------------
   CORRECÇÃO v5.2: COR_VERDE e COR_VERMELHO estavam trocados
   no código original v5.0. Valores corrigidos:
     COR_VERDE    : era 0xC00028 (vermelho!) → agora 0x22C55E (verde)
     COR_VERMELHO : era 0x001840 (azul!)    → agora 0xFF3333 (vermelho)
   Restantes cores melhoradas para maior contraste no ST7789.
============================================================ */
#define COR_BRANCO      0xF0F0F0   /* branco ligeiramente quente       */
#define COR_PRETO       0x000000   /* fundo do ecrã                    */
#define COR_CINZENTO    0x707068   /* texto inactivo / subtítulos      */
#define COR_CINZ_CLARO  0xA0A098   /* cinzento claro para labels       */
#define COR_VERDE       0x22C55E   /* CORRIGIDO: verde online / MASTER */
#define COR_VERMELHO    0xFF3333   /* CORRIGIDO: vermelho offline/erro */
#define COR_AMARELO     0xFACC15   /* T activo / LIGHT ON              */
#define COR_LARANJA     0xFB923C   /* SAFE MODE                        */
#define COR_CIANO       0x22D3EE   /* Tc activo                        */
#define COR_VIOLETA     0xC084FC   /* velocidade activa                */
#define COR_SEPARADOR   0x2A2A2A   /* linhas divisórias                */
#define COR_FUNDO_CARD  0x0D0D0D   /* fundo dos cards de tráfego       */
#define COR_AZUL_INFO   0x60A5FA   /* informação neutra                */

/* Cores internas do canvas radar */
#define COR_RADAR_FUNDO  0x010601  /* preto esverdeado profundo        */
#define COR_RADAR_GRELHA 0x0F1F0F  /* linhas de grelha subtis          */
#define COR_RADAR_EIXO   0x1A3A1A  /* eixo central e base              */
#define COR_RADAR_SCAN   0x22C55E  /* linha de sweep (verde vivo)      */

/* ============================================================
   DIMENSÕES DO CANVAS DO RADAR
============================================================ */
#define RADAR_W         230   /* largura do canvas em px              */
#define RADAR_H          90   /* altura do canvas em px               */
#define RADAR_X_OFF       5   /* margem esquerda no ecrã              */
#define RADAR_Y_OFF     149   /* y de início do canvas no ecrã        */

/* ============================================================
   PONTEIROS PARA ELEMENTOS VISUAIS
   Todos inicializados por ui_create().
============================================================ */

/* Zona identidade (y: 0..35) */
static lv_obj_t *label_nome;
static lv_obj_t *label_badge;

/* Zona hardware (y: 37..92) */
static lv_obj_t *label_wifi;
static lv_obj_t *label_radar_st;
static lv_obj_t *label_neb_esq;
static lv_obj_t *label_neb_dir;
static lv_obj_t *label_dali;
static lv_obj_t *bar_dali;

/* Zona tráfego (y: 94..145) */
static lv_obj_t *label_T_val;
static lv_obj_t *label_Tc_val;
static lv_obj_t *label_vel_val;

/* Cards de tráfego — necessário para actualizar borda */
static lv_obj_t *card_T;
static lv_obj_t *card_Tc;
static lv_obj_t *card_vel;

/* Zona radar (y: 147..239) */
static lv_obj_t *canvas_radar;

/* Buffer do canvas — RGB565, 1 word por pixel */
static lv_color_t radar_buf[RADAR_W * RADAR_H];

/* Estado interno dos objectos radar */
static radar_obj_t s_radar_objs[RADAR_MAX_OBJ];
static uint8_t     s_radar_count = 0;


/* ============================================================
   FUNÇÕES INTERNAS DE APOIO
============================================================ */

/* ------------------------------------------------------------
   st7789_flush_cb
   Callback de flush — transfere buffer LVGL para ST7789 SPI.
------------------------------------------------------------ */
static void st7789_flush_cb(lv_disp_drv_t  *disp_drv,
                             const lv_area_t *area,
                             lv_color_t      *color_p)
{
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    st7789_draw_bitmap((uint16_t)area->x1, (uint16_t)area->y1,
                       (uint16_t)w, (uint16_t)h,
                       (const uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

/* ------------------------------------------------------------
   _separador
   Linha horizontal de 1px entre zonas do ecrã.
------------------------------------------------------------ */
static void _separador(lv_obj_t *pai, int y_pos)
{
    lv_obj_t *l = lv_obj_create(pai);
    lv_obj_set_size(l, LCD_H_RES - 10, 1);
    lv_obj_set_pos(l, 5, y_pos);
    lv_obj_set_style_bg_color(l, lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(l, 0, 0);
    lv_obj_set_style_radius(l, 0, 0);
    lv_obj_set_style_pad_all(l, 0, 0);
}

/* ------------------------------------------------------------
   _label_novo
   Cria lv_label com posição, cor e texto iniciais.
   Fonte padrão: Montserrat 12.
------------------------------------------------------------ */
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
    lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
    return l;
}

/* ============================================================
   FUNÇÕES DE DESENHO PIXEL DIRECTO NO BUFFER RADAR
   ------------------------------------------------------------
   Escrevem directamente em radar_buf[] para controlo total
   da cor sem artefactos do renderer LVGL em modo canvas.
============================================================ */

/* Escreve pixel sólido — ignora coordenadas fora do canvas */
static inline void _px_set(int x, int y, lv_color_t cor)
{
    if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) return;
    radar_buf[y * RADAR_W + x] = cor;
}

/* Mistura pixel com alpha 0-255 usando lv_color_mix */
static inline void _px_blend(int x, int y,
                              lv_color_t cor, uint8_t alpha)
{
    if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) return;
    if (alpha == 0)   return;
    if (alpha == 255) { radar_buf[y * RADAR_W + x] = cor; return; }
    radar_buf[y * RADAR_W + x] =
        lv_color_mix(cor, radar_buf[y * RADAR_W + x], alpha);
}

/* Linha horizontal sólida — usada para grelha de distância */
static void _linha_h_px(int y, lv_color_t cor)
{
    if ((unsigned)y >= RADAR_H) return;
    for (int x = 0; x < RADAR_W; x++)
        radar_buf[y * RADAR_W + x] = cor;
}

/* Círculo preenchido com fade radial — centro mais brilhante */
static void _circulo_px(int cx, int cy, int r, lv_color_t cor)
{
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 > r2) continue;
            uint8_t alpha = (uint8_t)(255 - (d2 * 75) / (r2 + 1));
            _px_blend(cx + dx, cy + dy, cor, alpha);
        }
    }
}

/* Anel oco com fade do interior para exterior — halo pulsante */
static void _halo_px(int cx, int cy,
                     int r_int, int r_ext,
                     lv_color_t cor, uint8_t alpha_base)
{
    int ri2  = r_int * r_int;
    int re2  = r_ext * r_ext;
    int span = re2 - ri2 + 1;

    for (int dy = -r_ext; dy <= r_ext; dy++) {
        for (int dx = -r_ext; dx <= r_ext; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 < ri2 || d2 > re2) continue;
            uint8_t a = (uint8_t)((uint32_t)alpha_base
                                  * (re2 - d2) / span);
            _px_blend(cx + dx, cy + dy, cor, a);
        }
    }
}

/* ------------------------------------------------------------
   _radar_mm_to_px
   Converte coordenadas HLK-LD2450 (mm) para pixels no canvas.

   Sistema do sensor:
     X_mm : lateral (neg=esquerda, pos=direita), centrado em 0
     Y_mm : frontal (0=sensor, RADAR_MAX_MM=limite alcance)

   Mapeamento canvas (RADAR_W × RADAR_H):
     px_x : X=0 → centro horizontal do canvas
     px_y : Y=0 → fundo do canvas (sensor); aumenta para cima
------------------------------------------------------------ */
static void _radar_mm_to_px(int x_mm, int y_mm,
                             lv_coord_t *px_x, lv_coord_t *px_y)
{
    *px_x = (lv_coord_t)(RADAR_W / 2
            + ((float)x_mm / RADAR_XSPAN_MM)
            * (RADAR_W / 2 - 4));

    *px_y = (lv_coord_t)(RADAR_H - 4
            - ((float)y_mm / RADAR_MAX_MM)
            * (RADAR_H - 8));

    if (*px_x < 0)        *px_x = 0;
    if (*px_x >= RADAR_W) *px_x = RADAR_W - 1;
    if (*px_y < 0)        *px_y = 0;
    if (*px_y >= RADAR_H) *px_y = RADAR_H - 1;
}

/* ------------------------------------------------------------
   _radar_redraw
   ------------------------------------------------------------
   Redesenha o canvas completo do radar pixel a pixel.

   Pipeline (painter's algorithm):
     1.  Fundo preto-esverdeado profundo
     2.  Arcos semicirculares de alcance (1m, 3.5m, 7m)
     3.  Raios guia do sensor (35°, 65°, 90°, 115°, 145°)
     4.  Grelha horizontal de distância (1m a 7m)
     5.  Linha de base do sensor
     6.  Sweep animado (3°/frame) com sector de fade de 35°
     7.  Rasto de cada objecto com opacidade crescente
     8.  Halo duplo pulsante de cada objecto
     9.  Ponto do alvo com highlight de profundidade
    10.  Ponto verde do sensor (centro da base)
------------------------------------------------------------ */
static void _radar_redraw(void)
{
    if (!canvas_radar) return;

    /* Cores pré-calculadas por frame */
    lv_color_t C_FUNDO  = lv_color_hex(0x010601);
    lv_color_t C_ARCO   = lv_color_hex(0x0D2A0D);
    lv_color_t C_EIXO   = lv_color_hex(0x0D2A0D);
    lv_color_t C_GR1    = lv_color_hex(0x0A1A0A);
    lv_color_t C_GR2    = lv_color_hex(0x070E07);
    lv_color_t C_SWEEP  = lv_color_hex(0x22C55E);
    lv_color_t C_VRM    = lv_color_hex(0xFF3333);
    lv_color_t C_VRM_HL = lv_color_hex(0xFFAAAA);
    lv_color_t C_VRD    = lv_color_hex(0x22C55E);
    lv_color_t C_VRD_HL = lv_color_hex(0xAAFFAA);

    /* 1. Fundo */
    for (int i = 0; i < RADAR_W * RADAR_H; i++)
        radar_buf[i] = C_FUNDO;

    /* 2. Arcos semicirculares de alcance */
    int cx_s  = RADAR_W / 2;
    int cy_s  = RADAR_H - 1;
    int r_max = (int)((float)(RADAR_H - 3) * 1.85f);

    const float arc_f[3] = {1.0f/7.0f, 3.5f/7.0f, 1.0f};
    for (int a = 0; a < 3; a++) {
        int r = (int)(arc_f[a] * r_max);
        for (int dx = -r; dx <= r; dx++) {
            int dy2 = r*r - dx*dx;
            if (dy2 < 0) continue;
            int dy = (int)sqrtf((float)dy2);
            _px_blend(cx_s + dx, cy_s - dy, C_ARCO,
                      a == 2 ? 100u : 60u);
        }
    }

    /* 3. Raios guia */
    const int raios[5] = {35, 65, 90, 115, 145};
    for (int ri = 0; ri < 5; ri++) {
        float ang = (float)raios[ri] * 3.14159f / 180.0f;
        float fdx =  cosf(ang);
        float fdy = -sinf(ang);
        uint8_t opa = (raios[ri] == 90) ? 70u : 35u;
        for (float t = 2.0f; t < (float)r_max; t += 1.0f) {
            int x = cx_s + (int)(t * fdx);
            int y = cy_s + (int)(t * fdy);
            if ((unsigned)x >= RADAR_W ||
                (unsigned)y >= RADAR_H) break;
            _px_blend(x, y, C_EIXO, opa);
        }
    }

    /* 4. Grelha horizontal de distância (1m a 7m) */
    for (int m = 1; m <= 7; m++) {
        lv_coord_t py, dummy;
        _radar_mm_to_px(0, m * 1000, &dummy, &py);
        if (py < 0 || py >= RADAR_H) continue;
        lv_color_t cg = (m % 2 == 0) ? C_GR1 : C_GR2;
        for (int x = 0; x < RADAR_W; x++)
            _px_blend(x, (int)py, cg, 110u);
    }

    /* 5. Linha de base do sensor */
    _linha_h_px(RADAR_H - 1, C_EIXO);
    _linha_h_px(RADAR_H - 2, lv_color_hex(0x0A1A0A));

    /* 6. Sweep animado — MELHORIA v5.2: sector de fade 35° */
    static uint16_t s_sweep_tick = 0;
    s_sweep_tick = (uint16_t)((s_sweep_tick + 3u) % 180u);

    float sw_ang = (float)s_sweep_tick * 3.14159f / 180.0f;
    float sw_dx  =  cosf(sw_ang);
    float sw_dy  = -sinf(sw_ang);

    /* Sector de fade atrás do sweep (35 graus) */
    for (int back = 1; back <= 35; back++) {
        int bd = ((int)s_sweep_tick - back + 360) % 180;
        float bang = (float)bd * 3.14159f / 180.0f;
        float bdx  =  cosf(bang);
        float bdy  = -sinf(bang);
        uint8_t fa = (uint8_t)(42 - back);
        if (fa == 0) break;
        for (float t = 2.0f; t < (float)r_max; t += 1.5f) {
            int x = cx_s + (int)(t * bdx);
            int y = cy_s + (int)(t * bdy);
            if ((unsigned)x >= RADAR_W ||
                (unsigned)y >= RADAR_H) break;
            _px_blend(x, y, C_SWEEP, fa);
        }
    }
    /* Linha principal do sweep */
    for (float t = 2.0f; t < (float)r_max; t += 1.0f) {
        int x = cx_s + (int)(t * sw_dx);
        int y = cy_s + (int)(t * sw_dy);
        if ((unsigned)x >= RADAR_W ||
            (unsigned)y >= RADAR_H) break;
        uint8_t fade = (uint8_t)(210u - (uint8_t)(t * 1.5f));
        if (fade < 50u) fade = 50u;
        _px_blend(x, y, C_SWEEP, fade);
    }

    /* 7 + 8 + 9. Objectos detectados */
    for (int i = 0; i < s_radar_count; i++) {
        const radar_obj_t *obj = &s_radar_objs[i];
        lv_coord_t px, py;
        _radar_mm_to_px(obj->x_mm, obj->y_mm, &px, &py);

        /* 7. Rasto com opacidade crescente do mais antigo */
        for (int t = 0; t < obj->trail_len; t++) {
            lv_coord_t tx, ty;
            _radar_mm_to_px(obj->trail_x[t], obj->trail_y[t],
                            &tx, &ty);
            int     r_tr = 1 + (t * 2) / (obj->trail_len + 1);
            uint8_t a_tr = (uint8_t)(35u
                           + (uint8_t)(t * 90u / obj->trail_len));
            lv_color_t c_tr = lv_color_make(
                (uint8_t)(220u * a_tr >> 8), 0u, 0u);
            _circulo_px((int)tx, (int)ty, r_tr, c_tr);
        }

        /* 8. Halo duplo */
        _halo_px((int)px, (int)py, 5, 10, C_VRM, 50u);
        _halo_px((int)px, (int)py, 3,  6, C_VRM, 90u);

        /* 9. Ponto principal com highlight */
        _circulo_px((int)px, (int)py, 3, C_VRM);
        _px_blend((int)px - 1, (int)py - 1, C_VRM_HL, 210u);
        _px_blend((int)px,     (int)py - 1, C_VRM_HL, 160u);
        _px_blend((int)px - 1, (int)py,     C_VRM_HL, 110u);
    }

    /* 10. Ponto do sensor — verde vivo, centro da base */
    _halo_px(cx_s, cy_s, 4, 9, C_VRD, 55u);
    _circulo_px(cx_s, cy_s, 3, C_VRD);
    _px_blend(cx_s - 1, cy_s - 1, C_VRD_HL, 210u);
    _px_blend(cx_s,     cy_s - 1, C_VRD_HL, 150u);

    /* Força redraw do LVGL */
    lv_obj_invalidate(canvas_radar);
}


/* ============================================================
   ui_create
   ------------------------------------------------------------
   Cria todos os elementos visuais do ecrã 240×240.
   Chamada uma única vez em display_manager_init().
============================================================ */
static void ui_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COR_PRETO),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* ========================================================
       ZONA IDENTIDADE  (y: 0..35)
       Linha 1: "Poste Inteligente" — cinzento, fonte 10
       Linha 2: ↑ + nome NVS — branco, fonte 14
       Badge  : modo FSM — alinhado à direita
    ======================================================== */

    lv_obj_t *label_sub = lv_label_create(scr);
    lv_label_set_text(label_sub, "Poste Inteligente");
    lv_obj_set_style_text_color(label_sub,
                                lv_color_hex(COR_CINZENTO), 0);
    lv_obj_set_style_text_font(label_sub,
                               &lv_font_montserrat_10, 0);
    lv_obj_align(label_sub, LV_ALIGN_TOP_LEFT, 8, 3);

    label_nome = lv_label_create(scr);
    lv_label_set_text_fmt(label_nome,
                          LV_SYMBOL_UP " %s", post_get_name());
    lv_obj_set_style_text_color(label_nome,
                                lv_color_hex(COR_BRANCO), 0);
    lv_obj_set_style_text_font(label_nome,
                               &lv_font_montserrat_14, 0);
    lv_obj_align(label_nome, LV_ALIGN_TOP_LEFT, 8, 15);

    /* Badge FSM — MELHORIA v5.2: padding consistente */
    label_badge = lv_label_create(scr);
    lv_label_set_text(label_badge, "IDLE");
    lv_obj_set_style_text_color(label_badge,
                                lv_color_hex(COR_CINZENTO), 0);
    lv_obj_set_style_text_font(label_badge,
                               &lv_font_montserrat_12, 0);
    lv_obj_set_style_bg_color(label_badge,
                              lv_color_hex(0x1C1C1C), 0);
    lv_obj_set_style_bg_opa(label_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(label_badge, 5, 0);
    lv_obj_set_style_pad_hor(label_badge, 7, 0);
    lv_obj_set_style_pad_ver(label_badge, 3, 0);
    lv_obj_set_style_border_color(label_badge,
                                  lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_border_width(label_badge, 1, 0);
    lv_obj_align(label_badge, LV_ALIGN_TOP_RIGHT, -8, 10);

    _separador(scr, 36);

    /* ========================================================
       ZONA HARDWARE  (y: 37..92)
       Linha 1 (y=39): WiFi à esquerda | Radar à direita
       Linha 2 (y=57): Vizinho Esq | divisor | Vizinho Dir
       Linha 3 (y=76): DALI texto | barra proporcional
    ======================================================== */

    label_wifi = _label_novo(scr, 8, 39,
                             COR_VERMELHO, "WiFi: OFF");

    label_radar_st = _label_novo(scr, 140, 39,
                                 COR_CINZENTO, "Radar: ---");

    label_neb_esq = _label_novo(scr, 8, 57,
                                COR_CINZ_CLARO, "Esq: ---");

    /* Divisor vertical central entre vizinhos */
    lv_obj_t *div_neb = lv_obj_create(scr);
    lv_obj_set_size(div_neb, 1, 12);
    lv_obj_set_pos(div_neb, LCD_H_RES / 2, 59);
    lv_obj_set_style_bg_color(div_neb,
                              lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_bg_opa(div_neb, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(div_neb, 0, 0);
    lv_obj_set_style_pad_all(div_neb, 0, 0);

    label_neb_dir = _label_novo(scr,
                                LCD_H_RES / 2 + 4, 57,
                                COR_CINZ_CLARO, "Dir: ---");

    label_dali = _label_novo(scr, 8, 76,
                             COR_CINZENTO, "DALI:  0%");

    /* Barra de brilho DALI */
    bar_dali = lv_bar_create(scr);
    lv_obj_set_size(bar_dali, 88, 7);
    lv_obj_set_pos(bar_dali, 144, 78);
    lv_bar_set_range(bar_dali, 0, 100);
    lv_bar_set_value(bar_dali, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_dali,
                              lv_color_hex(0x2A2A2A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_dali,
                              lv_color_hex(COR_AMARELO),
                              LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_dali, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_dali, 3, LV_PART_INDICATOR);

    _separador(scr, 93);

    /* ========================================================
       ZONA TRÁFEGO  (y: 94..145)
       3 cards lado a lado: T | Tc | km/h
       MELHORIA v5.2: referências aos cards guardadas para
       actualizar borda quando valor > 0
    ======================================================== */
    const int CW  = 70;
    const int CH  = 46;
    const int CY  = 95;
    const int GAP =  5;
    const int CX0 =  5;
    const int CX1 = CX0 + CW + GAP;
    const int CX2 = CX1 + CW + GAP;

    const uint32_t COR_CARD[3] = {
        COR_AMARELO, COR_CIANO, COR_VIOLETA
    };
    const char *TITULO_CARD[3] = {"T (aqui)", "Tc (vem)", "km/h"};
    const int   CX_ARR[3]      = {CX0, CX1, CX2};
    lv_obj_t  **VAL_LABELS[3]  = {
        &label_T_val, &label_Tc_val, &label_vel_val
    };
    lv_obj_t  **CARDS[3] = {&card_T, &card_Tc, &card_vel};

    for (int i = 0; i < 3; i++) {
        lv_obj_t *card = lv_obj_create(scr);
        lv_obj_set_size(card, CW, CH);
        lv_obj_set_pos(card, CX_ARR[i], CY);
        lv_obj_set_style_bg_color(card,
                                  lv_color_hex(COR_FUNDO_CARD), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card,
                                      lv_color_hex(COR_SEPARADOR), 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 5, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        *CARDS[i] = card;

        /* Faixa de cabeçalho colorida (opacidade reduzida) */
        lv_obj_t *hdr = lv_obj_create(card);
        lv_obj_set_size(hdr, CW, 14);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr,
                                  lv_color_hex(COR_CARD[i]), 0);
        lv_obj_set_style_bg_opa(hdr, 45, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_pad_all(hdr, 0, 0);

        lv_obj_t *lbl_tit = lv_label_create(hdr);
        lv_label_set_text(lbl_tit, TITULO_CARD[i]);
        lv_obj_set_style_text_font(lbl_tit,
                                   &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(lbl_tit,
                                    lv_color_hex(COR_CARD[i]), 0);
        lv_obj_center(lbl_tit);

        /* Valor numérico grande */
        *VAL_LABELS[i] = lv_label_create(card);
        lv_label_set_text(*VAL_LABELS[i], "0");
        lv_obj_set_style_text_color(*VAL_LABELS[i],
                                    lv_color_hex(COR_CINZENTO), 0);
        lv_obj_set_style_text_font(*VAL_LABELS[i],
                                   &lv_font_montserrat_18, 0);
        lv_obj_align(*VAL_LABELS[i], LV_ALIGN_BOTTOM_MID, 0, -3);
    }

    _separador(scr, 146);

    /* ========================================================
       ZONA RADAR  (y: 147..239)
       lv_canvas 230×90 px com desenho directo em radar_buf[].
    ======================================================== */

    lv_obj_t *lbl_radar_tit = lv_label_create(scr);
    lv_label_set_text(lbl_radar_tit, "HLK-LD2450  X/Y mm");
    lv_obj_set_style_text_color(lbl_radar_tit,
                                lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_text_font(lbl_radar_tit,
                               &lv_font_montserrat_10, 0);
    lv_obj_set_pos(lbl_radar_tit, RADAR_X_OFF, 148);

    canvas_radar = lv_canvas_create(scr);
    lv_canvas_set_buffer(canvas_radar, radar_buf,
                         RADAR_W, RADAR_H,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(canvas_radar, RADAR_X_OFF, RADAR_Y_OFF);
    lv_obj_set_style_border_color(canvas_radar,
                                  lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_border_width(canvas_radar, 1, 0);
    lv_obj_set_style_radius(canvas_radar, 3, 0);

    /* Primeiro desenho — campo vazio com sweep inicial */
    _radar_redraw();

    ESP_LOGI(TAG, "Interface v5.2 criada | ID=%d | Nome=%s",
             post_get_id(), post_get_name());
}


/* ============================================================
   API PÚBLICA — CICLO DE VIDA
============================================================ */

void display_manager_init(void)
{
    ESP_LOGI(TAG, "A inicializar display v5.2 | ID=%d | %s",
             post_get_id(), post_get_name());

    st7789_init();
    lv_init();

    /* Buffer de rendering — 10 linhas para flush eficiente */
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t         buf[LCD_H_RES * 10];
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_H_RES * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = LCD_H_RES;
    disp_drv.ver_res  = LCD_V_RES;
    disp_drv.flush_cb = st7789_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    memset(s_radar_objs, 0, sizeof(s_radar_objs));
    s_radar_count = 0;

    /* Cria fila de mensagens thread-safe (v5.3) */
    s_fila = xQueueCreate(DM_FILA_CAP, sizeof(dm_msg_t));
    if (!s_fila)
        ESP_LOGE(TAG, "Falha ao criar fila de mensagens!");

    ui_create();

    ESP_LOGI(TAG, "Display v5.3 pronto (thread-safe via fila)");
}

void display_manager_tick(uint32_t ms)
{
    /* lv_tick_inc é thread-safe (apenas incremento atómico) */
    lv_tick_inc(ms);
}

/* ------------------------------------------------------------
   display_manager_task
   ------------------------------------------------------------
   ÚNICA função que toca em objectos LVGL.
   Chamar exclusivamente da main_task.

   Por ciclo:
     1. Drena toda a fila de mensagens e aplica ao LVGL
     2. Chama lv_timer_handler() para renderizar/flush
   
   Como s_fila usa xQueueReceive com timeout=0 (não bloqueante),
   esta função retorna imediatamente se não houver mensagens,
   garantindo que lv_timer_handler() corre sempre.
------------------------------------------------------------ */
void display_manager_task(void)
{
    if (!s_fila) { lv_timer_handler(); return; }

    /* Drena fila completa — processa TODAS as mensagens pendentes */
    dm_msg_t msg;
    while (xQueueReceive(s_fila, &msg, 0) == pdTRUE)
    {
        switch (msg.tipo)
        {
            /* ---- STATUS ---- */
            case DM_MSG_STATUS:
                if (!label_badge) break;
                lv_label_set_text(label_badge, msg.st.status);
                {
                    uint32_t cor_txt = COR_CINZENTO;
                    uint32_t cor_bg  = 0x1C1C1C;
                    uint32_t cor_brd = COR_SEPARADOR;
                    const char *s = msg.st.status;
                    if      (strcmp(s, "LIGHT ON")  == 0) { cor_txt = COR_AMARELO;  cor_bg = 0x1E1600; cor_brd = 0x3A2A00; }
                    else if (strcmp(s, "SAFE MODE") == 0) { cor_txt = COR_LARANJA;  cor_bg = 0x1E0E00; cor_brd = 0x3A1A00; }
                    else if (strcmp(s, "MASTER")    == 0) { cor_txt = COR_VERDE;    cor_bg = 0x001A08; cor_brd = 0x003A10; }
                    else if (strcmp(s, "AUTONOMO")  == 0) { cor_txt = COR_VERMELHO; cor_bg = 0x1A0000; cor_brd = 0x3A0000; }
                    lv_obj_set_style_text_color(label_badge,  lv_color_hex(cor_txt), 0);
                    lv_obj_set_style_bg_color(label_badge,    lv_color_hex(cor_bg),  0);
                    lv_obj_set_style_border_color(label_badge,lv_color_hex(cor_brd), 0);
                }
                break;

            /* ---- WIFI ---- */
            case DM_MSG_WIFI:
                if (!label_wifi) break;
                if (msg.wifi.connected) {
                    char buf[36];
                    snprintf(buf, sizeof(buf), "WiFi: ON  %s",
                             msg.wifi.ip[0] ? msg.wifi.ip : "---");
                    lv_label_set_text(label_wifi, buf);
                    lv_obj_set_style_text_color(label_wifi, lv_color_hex(COR_VERDE), 0);
                } else {
                    lv_label_set_text(label_wifi, "WiFi: OFF");
                    lv_obj_set_style_text_color(label_wifi, lv_color_hex(COR_VERMELHO), 0);
                }
                break;

            /* ---- HARDWARE ---- */
            case DM_MSG_HARDWARE:
                if (label_radar_st) {
                    lv_label_set_text(label_radar_st,
                        msg.hw.radar_ok ? "Radar: OK" : "Radar: ERR");
                    lv_obj_set_style_text_color(label_radar_st,
                        lv_color_hex(msg.hw.radar_ok ? COR_VERDE : COR_VERMELHO), 0);
                }
                if (label_dali) {
                    char buf[14];
                    snprintf(buf, sizeof(buf), "DALI: %3d%%", msg.hw.brightness);
                    lv_label_set_text(label_dali, buf);
                    lv_obj_set_style_text_color(label_dali,
                        lv_color_hex(msg.hw.brightness > LIGHT_MIN ? COR_AMARELO : COR_CINZENTO), 0);
                }
                if (bar_dali)
                    lv_bar_set_value(bar_dali, (int)msg.hw.brightness, LV_ANIM_ON);
                break;

            /* ---- TRÁFEGO ---- */
            case DM_MSG_TRAFFIC:
                if (label_T_val) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", msg.traf.T);
                    lv_label_set_text(label_T_val, buf);
                    lv_obj_set_style_text_color(label_T_val,
                        lv_color_hex(msg.traf.T > 0 ? COR_AMARELO : COR_CINZENTO), 0);
                    if (card_T)
                        lv_obj_set_style_border_color(card_T,
                            lv_color_hex(msg.traf.T > 0 ? COR_AMARELO : COR_SEPARADOR), 0);
                }
                if (label_Tc_val) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", msg.traf.Tc);
                    lv_label_set_text(label_Tc_val, buf);
                    lv_obj_set_style_text_color(label_Tc_val,
                        lv_color_hex(msg.traf.Tc > 0 ? COR_CIANO : COR_CINZENTO), 0);
                    if (card_Tc)
                        lv_obj_set_style_border_color(card_Tc,
                            lv_color_hex(msg.traf.Tc > 0 ? COR_CIANO : COR_SEPARADOR), 0);
                }
                break;

            /* ---- VELOCIDADE ---- */
            case DM_MSG_SPEED:
                if (label_vel_val) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", msg.spd.speed);
                    lv_label_set_text(label_vel_val, buf);
                    lv_obj_set_style_text_color(label_vel_val,
                        lv_color_hex(msg.spd.speed > 0 ? COR_VIOLETA : COR_CINZENTO), 0);
                    if (card_vel)
                        lv_obj_set_style_border_color(card_vel,
                            lv_color_hex(msg.spd.speed > 0 ? COR_VIOLETA : COR_SEPARADOR), 0);
                }
                break;

            /* ---- VIZINHOS ---- */
            case DM_MSG_NEIGHBORS:
                if (label_neb_esq) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "E:%s %s",
                             msg.neb.nebL[0] ? msg.neb.nebL : "---",
                             msg.neb.leftOk ? "OK" : "OFF");
                    lv_label_set_text(label_neb_esq, buf);
                    lv_obj_set_style_text_color(label_neb_esq,
                        lv_color_hex(msg.neb.leftOk ? COR_VERDE : COR_VERMELHO), 0);
                }
                if (label_neb_dir) {
                    char buf[32];
                    snprintf(buf, sizeof(buf), "D:%s %s",
                             msg.neb.nebR[0] ? msg.neb.nebR : "---",
                             msg.neb.rightOk ? "OK" : "OFF");
                    lv_label_set_text(label_neb_dir, buf);
                    lv_obj_set_style_text_color(label_neb_dir,
                        lv_color_hex(msg.neb.rightOk ? COR_VERDE : COR_VERMELHO), 0);
                }
                break;

            /* ---- RADAR ---- */
            case DM_MSG_RADAR:
                s_radar_count = msg.radar.count;
                if (msg.radar.count > 0)
                    memcpy(s_radar_objs, msg.radar.objs,
                           msg.radar.count * sizeof(radar_obj_t));
                else
                    memset(s_radar_objs, 0, sizeof(s_radar_objs));
                _radar_redraw();
                break;

            default:
                break;
        }
    }

    /* Renderiza LVGL — sempre, mesmo que não haja mensagens */
    lv_timer_handler();
}


/* ============================================================
   API PÚBLICA — ACTUALIZAÇÃO DE ESTADO
   ------------------------------------------------------------
   TODAS as funções abaixo são NON-BLOCKING e THREAD-SAFE.
   Apenas empacotam os dados numa dm_msg_t e fazem xQueueSend().
   Nunca tocam em objectos LVGL directamente.
   O LVGL só é acedido em display_manager_task() (main_task).
============================================================ */

void display_manager_set_status(const char *status)
{
    if (!s_fila || !status) return;
    dm_msg_t msg = { .tipo = DM_MSG_STATUS };
    strncpy(msg.st.status, status, sizeof(msg.st.status) - 1);
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_leader(bool is_leader)
{
    display_manager_set_status(is_leader ? "MASTER" : "IDLE");
}

void display_manager_set_wifi(bool connected, const char *ip)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_WIFI };
    msg.wifi.connected = connected;
    if (ip && ip[0])
        strncpy(msg.wifi.ip, ip, sizeof(msg.wifi.ip) - 1);
    else
        msg.wifi.ip[0] = '\0';
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_hardware(bool radar_ok, uint8_t brightness)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_HARDWARE };
    msg.hw.radar_ok   = radar_ok;
    msg.hw.brightness = brightness;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_traffic(int T, int Tc)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_TRAFFIC };
    msg.traf.T  = T;
    msg.traf.Tc = Tc;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_speed(int speed)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_SPEED };
    msg.spd.speed = speed;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_neighbors(const char *nebL,
                                   const char *nebR,
                                   bool        leftOk,
                                   bool        rightOk)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_NEIGHBORS };
    if (nebL && nebL[0])
        strncpy(msg.neb.nebL, nebL, sizeof(msg.neb.nebL) - 1);
    else
        msg.neb.nebL[0] = '\0';
    if (nebR && nebR[0])
        strncpy(msg.neb.nebR, nebR, sizeof(msg.neb.nebR) - 1);
    else
        msg.neb.nebR[0] = '\0';
    msg.neb.leftOk  = leftOk;
    msg.neb.rightOk = rightOk;
    xQueueSend(s_fila, &msg, 0);
}

void display_manager_set_radar(const radar_obj_t *objs,
                               uint8_t            count)
{
    if (!s_fila) return;
    dm_msg_t msg = { .tipo = DM_MSG_RADAR };
    msg.radar.count = (count > RADAR_MAX_OBJ) ? RADAR_MAX_OBJ : count;
    if (objs && msg.radar.count > 0)
        memcpy(msg.radar.objs, objs,
               msg.radar.count * sizeof(radar_obj_t));
    xQueueSend(s_fila, &msg, 0);
}