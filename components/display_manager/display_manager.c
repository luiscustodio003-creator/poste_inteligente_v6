/* ============================================================
   DISPLAY MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      display_manager.c
   @brief     Camada de apresentação LVGL + ST7789 — ecrã 240×240
   

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Descrição:
   ----------
   layout final aprovado. Substitui a ZONA GRÁFICOS
   por uma ZONA RADAR com visualização em tempo real dos objectos
   detectados pelo HLK-LD2450, usando lv_canvas com desenho
   directo em pixels. Os vizinhos passam para a ZONA HARDWARE,
   libertando os 44 px da zona inferior inteiramente para o radar.

   Layout do ecrã v5.0 (alturas em pixels):
   ------------------------------------------
     y=  0.. 35  → ZONA IDENTIDADE   (subtítulo + nome NVS + badge FSM)
     y= 36.. 36  → separador
     y= 37.. 92  → ZONA HARDWARE     (WiFi+Radar / Vizinhos / DALI+barra)
     y= 93.. 93  → separador
     y= 94..145  → ZONA TRÁFEGO      (cards T | Tc | km/h)
     y=146..146  → separador
     y=147..239  → ZONA RADAR        (lv_canvas 230×90 px, HLK-LD2450 X/Y)

   Alterações v4.1 → v5.0:
   ------------------------
   1. ZONA VIZINHOS removida do rodapé
   2. ZONA HARDWARE expandida: linha 1=WiFi+Radar, linha 2=Vizinhos,
      linha 3=DALI+barra
   3. ZONA GRÁFICOS (lv_chart) removida
   4. ZONA RADAR adicionada — lv_canvas 230×90 px com:
        - Grelha de distância (1m..7m) e faixas de rodagem
        - Até 3 objectos: ponto vermelho + halo + rasto
        - Ponto verde no centro da base (sensor/poste)
        - Redesenho completo a cada display_manager_set_radar()
   5. display_manager_set_neighbors() actualiza labels na ZONA HARDWARE
   6. display_manager_set_radar() API NOVA — recebe array de objectos
   7. Todas as funções anteriores (set_status, set_wifi, set_hardware,
      set_traffic, set_speed) mantidas com assinaturas inalteradas

   Dependências:
   -------------
   - st7789.h        : driver SPI do display físico
   - system_config.h : LCD_H_RES, LCD_V_RES, LIGHT_MIN
   - hw_config.h     : LCD_PIN_*
   - post_config.h   : post_get_name(), post_get_id()
   - lvgl            : v8.3.x  (LV_USE_CANVAS=1 em lv_conf.h)

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

/* ============================================================
   ETIQUETA DE LOG
============================================================ */
static const char *TAG = "DISP_MGR";

/* ============================================================
   PALETA DE CORES — hex RGB888
   Mantida idêntica às versões anteriores para consistência.
============================================================ */
#define COR_BRANCO      0xFFFFFF
#define COR_PRETO       0x000000
#define COR_CINZENTO    0x888780
#define COR_VERDE       0xC00028
#define COR_VERMELHO    0x001840
#define COR_AMARELO     0xFACC15
#define COR_LARANJA     0xFFBF00
#define COR_CIANO       0x22D3EE
#define COR_VIOLETA     0xA78BFA
#define COR_SEPARADOR   0x333333
#define COR_FUNDO_CARD  0x0D0D0D

/* Cores específicas do radar (componentes RGB separados para
   lv_canvas_set_px_color que usa lv_color_t directamente)    */
#define COR_RADAR_FUNDO  0x030A03   /* verde muito escuro          */
#define COR_RADAR_GRELHA 0x0F1F0F   /* linhas de grelha            */
#define COR_RADAR_EIXO   0x1A3A1A   /* eixo central e base         */
#define COR_RADAR_SCAN   0x22C55E   /* linha de scan (verde)       */

/* ============================================================
   DIMENSÕES DO CANVAS DO RADAR (privadas — não exportar)
   RADAR_MAX_MM e RADAR_XSPAN_MM estão em display_manager.h
   pois o main.c também precisa deles para os objectos simulados.
============================================================ */
#define RADAR_W         230   /* largura do canvas em px     */
#define RADAR_H          90   /* altura do canvas em px      */
#define RADAR_X_OFF       5   /* margem esquerda no ecrã     */
#define RADAR_Y_OFF     149   /* y de início do canvas       */

/* ============================================================
   PONTEIROS PARA ELEMENTOS VISUAIS
   Todos inicializados por ui_create().
============================================================ */

/* --- Zona identidade (y: 0..35) --- */
static lv_obj_t *label_nome;       /* LV_SYMBOL_UP + post_get_name()  */
static lv_obj_t *label_badge;      /* Badge de modo FSM               */

/* --- Zona hardware (y: 37..92) --- */
static lv_obj_t *label_wifi;       /* "WiFi: ON  x.x.x.x"            */
static lv_obj_t *label_radar_st;   /* "Radar: OK" / "Radar: ERR"      */
static lv_obj_t *label_neb_esq;    /* Vizinho esquerdo (linha 2)       */
static lv_obj_t *label_neb_dir;    /* Vizinho direito  (linha 2)       */
static lv_obj_t *label_dali;       /* "DALI: 75%"                     */
static lv_obj_t *bar_dali;         /* Barra proporcional 0-100%       */

/* --- Zona tráfego (y: 94..145) --- */
static lv_obj_t *label_T_val;      /* Valor numérico de T             */
static lv_obj_t *label_Tc_val;     /* Valor numérico de Tc            */
static lv_obj_t *label_vel_val;    /* Velocidade km/h                 */

/* --- Zona radar (y: 147..239) --- */
static lv_obj_t *canvas_radar;     /* lv_canvas para desenho directo  */

/* Buffer do canvas — RGB565, 1 palavra por pixel */
static lv_color_t radar_buf[RADAR_W * RADAR_H];

/* Estado interno dos objectos no radar — actualizado por
   display_manager_set_radar() e relido em _radar_redraw()   */
static radar_obj_t s_radar_objs[RADAR_MAX_OBJ];
static uint8_t     s_radar_count = 0;


/* ============================================================
   FUNÇÕES INTERNAS DE APOIO
============================================================ */

/* ------------------------------------------------------------
   st7789_flush_cb
   Callback de flush exigido pelo LVGL. Transfere o buffer de
   rendering para o display ST7789 via SPI DMA.
------------------------------------------------------------ */
static void st7789_flush_cb(lv_disp_drv_t  *disp_drv,const lv_area_t *area,lv_color_t *color_p)
{
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    st7789_draw_bitmap(area->x1, area->y1, w, h,(const uint16_t *)color_p);
    lv_disp_flush_ready(disp_drv);
}

/* ------------------------------------------------------------
   _separador
   Linha horizontal fina de 1 px entre zonas do ecrã.
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
static lv_obj_t *_label_novo(lv_obj_t   *pai,int x,int y,uint32_t cor,const char *texto)
{
    lv_obj_t *l = lv_label_create(pai);
    lv_label_set_text(l, texto);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_text_color(l, lv_color_hex(cor), 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
    return l;
}

/* ------------------------------------------------------------
   _radar_mm_to_px
   Converte coordenadas reais do HLK-LD2450 (mm) para pixel
   no canvas do radar.

   Sistema de coordenadas do sensor:
     X_mm : posição lateral (negativo=esquerda, positivo=direita)
     Y_mm : distância frontal (0=sensor, 7000=limite alcance)

   No canvas:
     px_x : centro horizontal = sensor (X=0)
     px_y : base = sensor (Y=0), topo = alcance máximo
------------------------------------------------------------ */
static void _radar_mm_to_px(int x_mm, int y_mm,lv_coord_t *px_x, lv_coord_t *px_y)
{
    /* Escala X: ±RADAR_XSPAN_MM → 0..RADAR_W */
    *px_x = (lv_coord_t)(RADAR_W / 2 +((float)x_mm / RADAR_XSPAN_MM) * (RADAR_W / 2 - 4));

    /* Escala Y: 0..RADAR_MAX_MM → RADAR_H..0 (invertido) */
    *px_y = (lv_coord_t)(RADAR_H - 4 -((float)y_mm / RADAR_MAX_MM) * (RADAR_H - 8));

    /* Limita ao interior do canvas */
    if (*px_x < 0)        *px_x = 0;
    if (*px_x >= RADAR_W) *px_x = RADAR_W - 1;
    if (*px_y < 0)        *px_y = 0;
    if (*px_y >= RADAR_H) *px_y = RADAR_H - 1;
}

/* ============================================================
   FUNÇÕES DE DESENHO PIXEL DIRECTO NO BUFFER RADAR
   ------------------------------------------------------------
   Estas funções escrevem directamente em radar_buf[] em vez
   de usar lv_canvas_draw_*. Isso é necessário porque o renderer
   de software do LVGL introduz cores lavadas e baixa resolução
   ao trabalhar em modo canvas RGB565.

   Todas as funções abaixo são internas ao módulo — prefixo _px_.
============================================================ */

/* ------------------------------------------------------------
   _px_set
   Escreve um pixel no buffer sem blending (cor sólida).
   Ignora silenciosamente coordenadas fora do canvas.
------------------------------------------------------------ */
static inline void _px_set(int x, int y, lv_color_t cor)
{
    if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) return;
    radar_buf[y * RADAR_W + x] = cor;
}

/* ------------------------------------------------------------
   _px_blend
   Mistura um pixel com o valor já no buffer usando alpha 0-255.

   Usa lv_color_mix() do LVGL que já trata correctamente os
   campos RGB565 independentemente de LV_COLOR_16_SWAP.
   Internamente converte alpha de 0-255 para 0-255 (LV_OPA_*).

   alpha=0   → transparente (não escreve)
   alpha=255 → cor sólida   (sobrescreve)
------------------------------------------------------------ */
static inline void _px_blend(int x, int y,lv_color_t cor, uint8_t alpha)
{
    if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) return;
    if (alpha == 0)   return;
    if (alpha == 255) { radar_buf[y * RADAR_W + x] = cor; return; }

    /* lv_color_mix(fg, bg, ratio): ratio=255 → 100% fg
       Converte alpha 0-255 para ratio 0-255 directamente      */
    radar_buf[y * RADAR_W + x] = lv_color_mix(cor,radar_buf[y * RADAR_W + x],alpha);
}

/* ------------------------------------------------------------
   _linha_h_px
   Linha horizontal sólida — usada para grelha de distância.
------------------------------------------------------------ */
static void _linha_h_px(int y, lv_color_t cor)
{
    if ((unsigned)y >= RADAR_H) return;
    for (int x = 0; x < RADAR_W; x++)
        radar_buf[y * RADAR_W + x] = cor;
}

/* ------------------------------------------------------------
   _circulo_px
   Círculo preenchido com fade radial — centro mais brilhante,
   borda mais escura. Usa distância ao quadrado (sem sqrtf).
   Raio máximo útil: 8 px.
------------------------------------------------------------ */
static void _circulo_px(int cx, int cy, int r, lv_color_t cor)
{
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx*dx + dy*dy;
            if (d2 > r2) continue;
            /* Alpha mais alto no centro (d2=0) → 255,
               cai para ~180 na borda (d2=r2)           */
            uint8_t alpha = (uint8_t)(255 - (d2 * 75) / (r2 + 1));
            _px_blend(cx + dx, cy + dy, cor, alpha);
        }
    }
}

/* ------------------------------------------------------------
   _halo_px
   Anel oco entre r_int e r_ext com fade do interior para
   o exterior — simula o halo pulsante em volta de cada objecto.
   alpha_base: opacidade máxima do halo (0-255).
------------------------------------------------------------ */
static void _halo_px(int cx, int cy,int r_int, int r_ext,lv_color_t cor, uint8_t alpha_base)
{
    int ri2 = r_int * r_int;
    int re2 = r_ext * r_ext;
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
   _radar_redraw
   ------------------------------------------------------------
   Redesenha o canvas completo do radar escrevendo directamente
   no buffer radar_buf[] pixel a pixel.

   Pipeline de desenho (painter's algorithm):
     1.  Fundo preto-esverdeado profundo (#010601)
     2.  Arcos semicirculares de alcance (1m, 3.5m, 7m)
     3.  Raios guia do sensor (35°, 65°, 90°, 115°, 145°)
     4.  Grelha horizontal de distância (1m .. 7m)
     5.  Linha de base do sensor
     6.  Sweep line animada (3°/frame via s_sweep_tick)
     7.  Rasto de cada objecto (RADAR_TRAIL_MAX pontos)
     8.  Halo duplo pulsante de cada objecto
     9.  Ponto vermelho vivo com highlight branco
    10.  Ponto verde do sensor (centro da base) com halo

   Performance: 230×90 = 20 700 pixels por frame.
   A 100ms/frame é negligenciável no ESP32 @ 240 MHz.

   Chamada por display_manager_set_radar() a cada novo frame
   do HLK-LD2450 (ou da simulação).
------------------------------------------------------------ */
static void _radar_redraw(void)
{
    if (!canvas_radar) return;

    /* --------------------------------------------------------
       Cores pré-calculadas uma vez por frame
    -------------------------------------------------------- */
    lv_color_t C_FUNDO  = lv_color_hex(0x010601);  /* preto esverdeado  */
    lv_color_t C_ARCO   = lv_color_hex(0x0D1E0D);  /* arcos de alcance  */
    lv_color_t C_EIXO   = lv_color_hex(0x0D1E0D);  /* eixo e base       */
    lv_color_t C_GR1    = lv_color_hex(0x0A170A);  /* grelha pares      */
    lv_color_t C_GR2    = lv_color_hex(0x070E07);  /* grelha impares    */
    lv_color_t C_SWEEP  = lv_color_hex(0x22C55E);  /* sweep verde vivo  */
    lv_color_t C_VRM    = lv_color_hex(0xFF2222);  /* vermelho vivo     */
    lv_color_t C_VRM_HL = lv_color_hex(0xFFB0A0);  /* highlight obj.    */
    lv_color_t C_VRD    = lv_color_hex(0x22C55E);  /* sensor verde      */
    lv_color_t C_VRD_HL = lv_color_hex(0xC0FFC0);  /* highlight sensor  */

    /* --------------------------------------------------------
       1. Fundo
    -------------------------------------------------------- */
    for (int i = 0; i < RADAR_W * RADAR_H; i++)
        radar_buf[i] = C_FUNDO;

    /* --------------------------------------------------------
       2. Arcos semicirculares de alcance
       Origem: centro da base (cx, RADAR_H-1).
       Frações: 1/7, 3.5/7, 7/7 do alcance máximo.
    -------------------------------------------------------- */
    int cx_s  = RADAR_W / 2;
    int cy_s  = RADAR_H - 1;
    int r_max = (int)((float)(RADAR_H - 3) * 1.85f);

    const float arc_f[3] = {1.0f/7.0f, 3.5f/7.0f, 1.0f};
    for (int a = 0; a < 3; a++) {
        int r = (int)(arc_f[a] * r_max);
        /* Percorre pixels ao longo do semicírculo superior */
        for (int dx = -r; dx <= r; dx++) {
            int dy2 = r*r - dx*dx;
            if (dy2 < 0) continue;
            int dy = (int)sqrtf((float)dy2);
            _px_blend(cx_s + dx, cy_s - dy, C_ARCO,
                      a == 2 ? 90u : 55u);
        }
    }

    /* --------------------------------------------------------
       3. Raios guia (5 ângulos)
    -------------------------------------------------------- */
    const int raios[5] = {35, 65, 90, 115, 145};
    for (int ri = 0; ri < 5; ri++) {
        float ang = (float)raios[ri] * 3.14159f / 180.0f;
        float fdx = cosf(ang);
        float fdy = -sinf(ang);  /* y invertido no canvas */
        uint8_t opa = (raios[ri] == 90) ? 65u : 30u;
        for (float t = 2.0f; t < (float)r_max; t += 1.0f) {
            int x = cx_s + (int)(t * fdx);
            int y = cy_s + (int)(t * fdy);
            if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) break;
            _px_blend(x, y, C_EIXO, opa);
        }
    }

    /* --------------------------------------------------------
       4. Grelha horizontal de distância (1m a 7m)
    -------------------------------------------------------- */
    for (int m = 1; m <= 7; m++) {
        lv_coord_t py, dummy;
        _radar_mm_to_px(0, m * 1000, &dummy, &py);
        if (py < 0 || py >= RADAR_H) continue;
        lv_color_t cg = (m % 2 == 0) ? C_GR1 : C_GR2;
        for (int x = 0; x < RADAR_W; x++)
            _px_blend(x, (int)py, cg, 100u);
    }

    /* --------------------------------------------------------
       5. Linha de base do sensor
    -------------------------------------------------------- */
    _linha_h_px(RADAR_H - 1, C_EIXO);
    _linha_h_px(RADAR_H - 2, lv_color_hex(0x0A1A0A));

    /* --------------------------------------------------------
       6. Sweep animado (3° por chamada = 1 volta em ~12s a 100ms)
          s_sweep_tick avança de 0 a 179 graus (semicírculo).
    -------------------------------------------------------- */
    static uint16_t s_sweep_tick = 0;
    s_sweep_tick = (uint16_t)((s_sweep_tick + 3u) % 180u);

    float sw_ang = (float)s_sweep_tick * 3.14159f / 180.0f;
    float sw_dx  = cosf(sw_ang);
    float sw_dy  = -sinf(sw_ang);

    /* Sector de fade atrás do sweep */
    for (int back = 1; back <= 28; back++) {
        int bd = ((int)s_sweep_tick - back + 360) % 180;
        float bang = (float)bd * 3.14159f / 180.0f;
        float bdx  = cosf(bang), bdy = -sinf(bang);
        uint8_t fa = (uint8_t)(35 - back);
        if (fa == 0) break;
        for (float t = 2.0f; t < (float)r_max; t += 1.5f) {
            int x = cx_s + (int)(t * bdx);
            int y = cy_s + (int)(t * bdy);
            if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) break;
            _px_blend(x, y, C_SWEEP, fa);
        }
    }
    /* Linha principal do sweep */
    for (float t = 2.0f; t < (float)r_max; t += 1.0f) {
        int x = cx_s + (int)(t * sw_dx);
        int y = cy_s + (int)(t * sw_dy);
        if ((unsigned)x >= RADAR_W || (unsigned)y >= RADAR_H) break;
        uint8_t fade = (uint8_t)(200u - (uint8_t)(t * 1.5f));
        if (fade < 40u) fade = 40u;
        _px_blend(x, y, C_SWEEP, fade);
    }

    /* --------------------------------------------------------
       7 + 8 + 9. Objectos detectados
    -------------------------------------------------------- */
    for (int i = 0; i < s_radar_count; i++) {
        const radar_obj_t *obj = &s_radar_objs[i];
        lv_coord_t px, py;
        _radar_mm_to_px(obj->x_mm, obj->y_mm, &px, &py);

        /* 7. Rasto com opacidade crescente do mais antigo ao mais recente */
        for (int t = 0; t < obj->trail_len; t++) {
            lv_coord_t tx, ty;
            _radar_mm_to_px(obj->trail_x[t], obj->trail_y[t], &tx, &ty);
            int        r_trail = 1 + (t * 2) / (obj->trail_len + 1);
            uint8_t    a_trail = (uint8_t)(35u
                                 + (uint8_t)(t * 90u / obj->trail_len));
            /* Cria cor de rasto proporcional ao índice */
            lv_color_t c_tr = lv_color_make(
                (uint8_t)(200u * a_trail >> 8), 0u, 0u);
            _circulo_px((int)tx, (int)ty, r_trail, c_tr);
        }

        /* 8. Halo duplo */
        _halo_px((int)px, (int)py, 5, 9,  C_VRM, 45u);
        _halo_px((int)px, (int)py, 3, 6,  C_VRM, 85u);

        /* 9. Ponto principal vermelho vivo */
        _circulo_px((int)px, (int)py, 3, C_VRM);
        /* Highlight branco subtil no canto superior-esquerdo */
        _px_blend((int)px - 1, (int)py - 1, C_VRM_HL, 200u);
        _px_blend((int)px,     (int)py - 1, C_VRM_HL, 150u);
        _px_blend((int)px - 1, (int)py,     C_VRM_HL, 100u);
    }

    /* --------------------------------------------------------
       10. Ponto do sensor — verde vivo, centro da base
    -------------------------------------------------------- */
    _halo_px(cx_s, cy_s, 4, 9,  C_VRD, 50u);
    _circulo_px(cx_s, cy_s, 3, C_VRD);
    _px_blend(cx_s - 1, cy_s - 1, C_VRD_HL, 200u);
    _px_blend(cx_s,     cy_s - 1, C_VRD_HL, 140u);

    /* --------------------------------------------------------
       Força refresh do LVGL — marca canvas como dirty
    -------------------------------------------------------- */
    lv_obj_invalidate(canvas_radar);
}


/* ============================================================
   ui_create
   ------------------------------------------------------------
   Cria todos os elementos visuais do ecrã 240×240.
   Chamada uma única vez em display_manager_init().

   Layout v5.0:
     y=  0 → ZONA IDENTIDADE  (36 px)
     y= 36 → separador
     y= 37 → ZONA HARDWARE    (56 px: WiFi+Radar / Vizinhos / DALI)
     y= 93 → separador
     y= 94 → ZONA TRÁFEGO     (52 px: cards T | Tc | km/h)
     y=146 → separador
     y=147 → ZONA RADAR       (93 px: lv_canvas)
============================================================ */
static void ui_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COR_PRETO), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* ----------------------------------------------------------
       ZONA IDENTIDADE (y: 0..35)
       Linha 1: "Poste Inteligente" — cinzento, fonte 10
       Linha 2: LV_SYMBOL_UP + post_get_name() — branco, fonte 14
       Badge : modo FSM — alinhado à direita
    ---------------------------------------------------------- */

    /* Subtítulo fixo */
    lv_obj_t *label_sub = lv_label_create(scr);
    lv_label_set_text(label_sub, "Poste Inteligente");
    lv_obj_set_style_text_color(label_sub,lv_color_hex(COR_CINZENTO), 0);
    lv_obj_set_style_text_font(label_sub,&lv_font_montserrat_10, 0);
    lv_obj_align(label_sub, LV_ALIGN_TOP_LEFT, 8, 3);

    /* Nome dinâmico da NVS com símbolo LVGL built-in */
    label_nome = lv_label_create(scr);
    lv_label_set_text_fmt(label_nome,LV_SYMBOL_UP " %s", post_get_name());
    lv_obj_set_style_text_color(label_nome,lv_color_hex(COR_BRANCO), 0);
    lv_obj_set_style_text_font(label_nome,&lv_font_montserrat_14, 0);
    lv_obj_align(label_nome, LV_ALIGN_TOP_LEFT, 8, 15);

    /* Badge de modo FSM */
    label_badge = lv_label_create(scr);
    lv_label_set_text(label_badge, "IDLE");
    lv_obj_set_style_text_color(label_badge,lv_color_hex(COR_CINZENTO), 0);
    lv_obj_set_style_text_font(label_badge,&lv_font_montserrat_14, 0);
    lv_obj_set_style_bg_color(label_badge,lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_bg_opa(label_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(label_badge, 4, 0);
    lv_obj_set_style_pad_hor(label_badge, 8, 0);
    lv_obj_set_style_pad_ver(label_badge, 2, 0);
    lv_obj_align(label_badge, LV_ALIGN_TOP_RIGHT, -8, 10);

    _separador(scr, 36);

    /* ----------------------------------------------------------
       ZONA HARDWARE (y: 37..92) — 3 linhas de 17 px cada
       Linha 1 (y=39): WiFi à esquerda | Radar à direita
       Linha 2 (y=56): Vizinho Esq | divisor | Vizinho Dir
       Linha 3 (y=73): ☀ DALI texto | barra proporcional
    ---------------------------------------------------------- */

    /* Linha 1 — WiFi */
    label_wifi = _label_novo(scr, 8, 39, COR_VERMELHO, "WiFi: OFF");

    /* Linha 1 — Radar (alinhado à direita, posição calculada) */
    label_radar_st = _label_novo(scr, 140, 39, COR_CINZENTO, "Radar: ---");

    /* Linha 2 — Vizinho esquerdo */
    label_neb_esq = _label_novo(scr, 8, 57, COR_VERDE, "Esq: ---");

    /* Divisor vertical central entre vizinhos */
    lv_obj_t *div_neb = lv_obj_create(scr);
    lv_obj_set_size(div_neb, 1, 12);
    lv_obj_set_pos(div_neb, LCD_H_RES / 2, 59);lv_obj_set_style_bg_color(div_neb,lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_bg_opa(div_neb, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(div_neb, 0, 0);
    lv_obj_set_style_pad_all(div_neb, 0, 0);

    /* Linha 2 — Vizinho direito */
    label_neb_dir = _label_novo(scr,LCD_H_RES / 2 + 4, 57,COR_VERDE, "Dir: ---");

    /* Linha 3 — Texto DALI */
    label_dali = _label_novo(scr, 8, 76, COR_VERDE, "DALI:  0%");

    /* Linha 3 — Barra de brilho DALI */
    bar_dali = lv_bar_create(scr);
    lv_obj_set_size(bar_dali, 88, 7);
    lv_obj_set_pos(bar_dali, 144, 78);
    lv_bar_set_range(bar_dali, 0, 100);
    lv_bar_set_value(bar_dali, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_dali,lv_color_hex(0x2A2A2A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar_dali,lv_color_hex(COR_AMARELO),LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_dali, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_dali, 3, LV_PART_INDICATOR);

    _separador(scr, 93);

    /* ----------------------------------------------------------
       ZONA TRÁFEGO (y: 94..145) — três cards lado a lado
       Card T    : amarelo — veículos confirmados pelo radar local
       Card Tc   : ciano   — veículos anunciados via UDP
       Card km/h : violeta — velocidade média detectada
    ---------------------------------------------------------- */
    const int CW  = 70;
    const int CH  = 46;
    const int CY  = 95;
    const int GAP =  5;
    const int CX0 =  5;
    const int CX1 = CX0 + CW + GAP;
    const int CX2 = CX1 + CW + GAP;

    const uint32_t COR_CARD[3] = {COR_AMARELO, COR_CIANO, COR_VIOLETA};
    const char *TITULO_CARD[3] = {"T (aqui)", "Tc (vem)", "km/h"};
    const int   CX_ARR[3]      = {CX0, CX1, CX2};
    lv_obj_t  **VAL_LABELS[3]  = {&label_T_val, &label_Tc_val,&label_vel_val};

    for (int i = 0; i < 3; i++) {
        /* Fundo do card */
        lv_obj_t *card = lv_obj_create(scr);
        lv_obj_set_size(card, CW, CH);
        lv_obj_set_pos(card, CX_ARR[i], CY);
        lv_obj_set_style_bg_color(card,lv_color_hex(COR_FUNDO_CARD), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card,lv_color_hex(COR_CARD[i]), 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 5, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        /* Faixa de cabeçalho colorida */
        lv_obj_t *hdr = lv_obj_create(card);
        lv_obj_set_size(hdr, CW, 14);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr,lv_color_hex(COR_CARD[i]), 0);
        lv_obj_set_style_bg_opa(hdr, 55, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_pad_all(hdr, 0, 0);

        /* Título do card */
        lv_obj_t *lbl_tit = lv_label_create(hdr);
        lv_label_set_text(lbl_tit, TITULO_CARD[i]);
        lv_obj_set_style_text_font(lbl_tit,&lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(lbl_tit,lv_color_hex(COR_CARD[i]), 0);
        lv_obj_center(lbl_tit);

        /* Valor numérico grande */
        *VAL_LABELS[i] = lv_label_create(card);
        lv_label_set_text(*VAL_LABELS[i], "0");
        lv_obj_set_style_text_color(*VAL_LABELS[i],lv_color_hex(COR_CINZENTO), 0);
        lv_obj_set_style_text_font(*VAL_LABELS[i],&lv_font_montserrat_18, 0);
        lv_obj_align(*VAL_LABELS[i], LV_ALIGN_BOTTOM_MID, 0, -3);
    }

    _separador(scr, 146);

    /* ----------------------------------------------------------
       ZONA RADAR (y: 147..239) — lv_canvas 230×90 px
       O canvas usa um buffer estático (radar_buf) em RGB565.
       O redesenho é feito por _radar_redraw() chamada sempre
       que display_manager_set_radar() recebe novos dados.
    ---------------------------------------------------------- */

    /* Título "HLK-LD2450" acima do canvas */
    lv_obj_t *lbl_radar_tit = lv_label_create(scr);
    lv_label_set_text(lbl_radar_tit, "HLK-LD2450  X/Y mm");
    lv_obj_set_style_text_color(lbl_radar_tit,lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_text_font(lbl_radar_tit,&lv_font_montserrat_10, 0);
    lv_obj_set_pos(lbl_radar_tit, RADAR_X_OFF, 148);

    /* Canvas propriamente dito */
    canvas_radar = lv_canvas_create(scr);
    lv_canvas_set_buffer(canvas_radar, radar_buf,RADAR_W, RADAR_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(canvas_radar, RADAR_X_OFF, RADAR_Y_OFF);

    /* Borda subtil à volta do canvas */
    lv_obj_set_style_border_color(canvas_radar,lv_color_hex(COR_SEPARADOR), 0);
    lv_obj_set_style_border_width(canvas_radar, 1, 0);
    lv_obj_set_style_radius(canvas_radar, 3, 0);

    /* Primeiro desenho — campo vazio */
    _radar_redraw();

    ESP_LOGI(TAG, "Interface v5.0 criada | ID=%d | Nome=%s",
             post_get_id(), post_get_name());
}


/* ============================================================
   FUNÇÕES PÚBLICAS — CICLO DE VIDA
============================================================ */

/* ------------------------------------------------------------
   display_manager_init
   ------------------------------------------------------------
   Inicializa ST7789, LVGL e interface v5.0.

   Ordem obrigatória em app_main():
     1. nvs_flash_init()       — NVS
     2. post_config_init()     — carrega nome da NVS
     3. display_manager_init() — este módulo
------------------------------------------------------------ */
void display_manager_init(void)
{
    ESP_LOGI(TAG, "A inicializar display v5.0 | ID=%d | %s",
             post_get_id(), post_get_name());

    st7789_init();
    lv_init();

    /* Buffer de rendering — 10 linhas × LCD_H_RES px */
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t         buf[LCD_H_RES * 10];
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_H_RES * 10);

    /* Registo do driver */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = LCD_H_RES;
    disp_drv.ver_res  = LCD_V_RES;
    disp_drv.flush_cb = st7789_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Inicializa estado do radar vazio */
    memset(s_radar_objs, 0, sizeof(s_radar_objs));
    s_radar_count = 0;

    ui_create();

    ESP_LOGI(TAG, "Display v5.0 pronto (radar activo)");
}

/* ------------------------------------------------------------
   display_manager_tick
   Incrementa ticker LVGL. Chamar a cada 1 ms.
------------------------------------------------------------ */
void display_manager_tick(uint32_t ms)
{
    lv_tick_inc(ms);
}

/* ------------------------------------------------------------
   display_manager_task
   Processa eventos LVGL. Chamar a cada 5-10 ms.
------------------------------------------------------------ */
void display_manager_task(void)
{
    lv_timer_handler();
}


/* ============================================================
   FUNÇÕES PÚBLICAS — ACTUALIZAÇÃO DE ESTADO
============================================================ */

/* ------------------------------------------------------------
   display_manager_set_status
   Actualiza badge de modo com cor correspondente ao estado.

   Mapeamento:
     "IDLE"      → cinzento  (sem actividade)
     "LIGHT ON"  → amarelo   (luz activa)
     "SAFE MODE" → laranja   (radar falhou)
     "MASTER"    → verde     (poste líder)
     "AUTONOMO"  → vermelho  (sem rede)
------------------------------------------------------------ */
void display_manager_set_status(const char *status)
{
    if (!label_badge || !status) return;

    lv_label_set_text(label_badge, status);

    uint32_t cor_txt = COR_CINZENTO;
    uint32_t cor_bg  = 0x1A1A1A;

    if      (strcmp(status, "LIGHT ON")  == 0) {
        cor_txt = COR_AMARELO; cor_bg = 0x2A1A00;
    }
    else if (strcmp(status, "SAFE MODE") == 0) {
        cor_txt = COR_LARANJA; cor_bg = 0x2A1500;
    }
    else if (strcmp(status, "MASTER")    == 0) {
        cor_txt = COR_VERDE;   cor_bg = 0x0A2A0A;
    }
    else if (strcmp(status, "AUTONOMO")  == 0) {
        cor_txt = COR_VERMELHO; cor_bg = 0x2A0A0A;
    }

    lv_obj_set_style_text_color(label_badge,
                                lv_color_hex(cor_txt), 0);
    lv_obj_set_style_bg_color(label_badge,
                              lv_color_hex(cor_bg), 0);
}

/* ------------------------------------------------------------
   display_manager_set_leader
   Atalho: true → "MASTER", false → "IDLE".
------------------------------------------------------------ */
void display_manager_set_leader(bool is_leader)
{
    display_manager_set_status(is_leader ? "MASTER" : "IDLE");
}

/* ------------------------------------------------------------
   display_manager_set_wifi
   Actualiza estado Wi-Fi e IP (linha 1 da zona hardware).
------------------------------------------------------------ */
void display_manager_set_wifi(bool connected, const char *ip)
{
    if (!label_wifi) return;

    if (connected) {
        char buf[36];
        snprintf(buf, sizeof(buf), "WiFi: ON %s",(ip && ip[0]) ? ip : "---");
        lv_label_set_text(label_wifi, buf);
        lv_obj_set_style_text_color(label_wifi,lv_color_hex(COR_VERDE), 0);
    } else {
        lv_label_set_text(label_wifi, "WiFi: OFF");
        lv_obj_set_style_text_color(label_wifi,lv_color_hex(COR_VERMELHO), 0);
    }
}

/* ------------------------------------------------------------
   display_manager_set_hardware
   Actualiza estado do radar e brilho DALI (linhas 1 e 3).
------------------------------------------------------------ */
void display_manager_set_hardware(bool radar_ok, uint8_t brightness)
{
    /* Radar */
    if (label_radar_st) {
        lv_label_set_text(label_radar_st,
                          radar_ok ? "Radar: OK" : "Radar: ERR");
        lv_obj_set_style_text_color(label_radar_st,
                                    lv_color_hex(radar_ok
                                        ? COR_VERDE
                                        : COR_VERMELHO), 0);
    }

    /* DALI — texto */
    if (label_dali) {
        char buf[14];
        snprintf(buf, sizeof(buf), "DALI: %3d%%", brightness);
        lv_label_set_text(label_dali, buf);
        lv_obj_set_style_text_color(label_dali,
                                    lv_color_hex(brightness > LIGHT_MIN
                                        ? COR_AMARELO
                                        : COR_CINZENTO), 0);
    }

    /* DALI — barra animada */
    if (bar_dali) {
        lv_bar_set_value(bar_dali, (int)brightness, LV_ANIM_ON);
    }
}

/* ------------------------------------------------------------
   display_manager_set_traffic
   Actualiza cards T e Tc na zona de tráfego.
   T=0 → cinzento; T>0 → cor activa.
------------------------------------------------------------ */
void display_manager_set_traffic(int T, int Tc)
{
    if (label_T_val) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", T);
        lv_label_set_text(label_T_val, buf);
        lv_obj_set_style_text_color(label_T_val,
                                    lv_color_hex(T > 0
                                        ? COR_AMARELO
                                        : COR_CINZENTO), 0);
    }

    if (label_Tc_val) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", Tc);
        lv_label_set_text(label_Tc_val, buf);
        lv_obj_set_style_text_color(label_Tc_val,
                                    lv_color_hex(Tc > 0
                                        ? COR_CIANO
                                        : COR_CINZENTO), 0);
    }
}

/* ------------------------------------------------------------
   display_manager_set_speed
   Actualiza card km/h na zona de tráfego.
   speed=0 → cinzento; speed>0 → violeta.
------------------------------------------------------------ */
void display_manager_set_speed(int speed)
{
    if (!label_vel_val) return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", speed);
    lv_label_set_text(label_vel_val, buf);
    lv_obj_set_style_text_color(label_vel_val,
                                lv_color_hex(speed > 0
                                    ? COR_VIOLETA
                                    : COR_CINZENTO), 0);
}

/* ------------------------------------------------------------
   display_manager_set_neighbors
   Actualiza vizinhos na linha 2 da ZONA HARDWARE.
   IP completo + estado OK/OFF.
   Online → verde; offline → vermelho.
------------------------------------------------------------ */
void display_manager_set_neighbors(const char *nebL,
                                   const char *nebR,
                                   bool        leftOk,
                                   bool        rightOk)
{
    /* Vizinho esquerdo */
    if (label_neb_esq) {
        char buf[32];
        snprintf(buf, sizeof(buf), "E:%s %s",
                 (nebL && nebL[0]) ? nebL : "---",
                 leftOk ? "OK" : "OFF");
        lv_label_set_text(label_neb_esq, buf);
        lv_obj_set_style_text_color(label_neb_esq,
                                    lv_color_hex(leftOk
                                        ? COR_VERDE
                                        : COR_VERMELHO), 0);
    }

    /* Vizinho direito */
    if (label_neb_dir) {
        char buf[32];
        snprintf(buf, sizeof(buf), "D:%s %s",
                 (nebR && nebR[0]) ? nebR : "---",
                 rightOk ? "OK" : "OFF");
        lv_label_set_text(label_neb_dir, buf);
        lv_obj_set_style_text_color(label_neb_dir,
                                    lv_color_hex(rightOk
                                        ? COR_VERDE
                                        : COR_VERMELHO), 0);
    }
}

/* ------------------------------------------------------------
   display_manager_set_radar
   API NOVA v5.0 — actualiza os objectos detectados no radar.

   Chamada pelo radar_manager sempre que chega um novo frame
   do HLK-LD2450 (byte[0]=0xAA).

   @param  objs   Array de radar_obj_t com posições X/Y em mm
                  e rasto dos últimos frames.
   @param  count  Número de objectos no array (0..RADAR_MAX_OBJ).
                  Se count=0, o radar fica vazio (sem pontos).
------------------------------------------------------------ */
void display_manager_set_radar(const radar_obj_t *objs, uint8_t count)
{
    if (!canvas_radar) return;

    /* Copia os dados para o estado interno */
    s_radar_count = (count > RADAR_MAX_OBJ) ? RADAR_MAX_OBJ : count;

    if (objs && s_radar_count > 0) {
        memcpy(s_radar_objs, objs,
               s_radar_count * sizeof(radar_obj_t));
    } else {
        /* Sem objectos — limpa estado */
        memset(s_radar_objs, 0, sizeof(s_radar_objs));
        s_radar_count = 0;
    }

    /* Redesenha o canvas imediatamente */
    _radar_redraw();
}