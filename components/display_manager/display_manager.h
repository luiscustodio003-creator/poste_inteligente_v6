/* ============================================================
   DISPLAY MANAGER — CABEÇALHO DO MÓDULO
   ------------------------------------------------------------
   @file      display_manager.h
   @brief     Interface pública do gestor de ecrã LVGL (v5.0)
   

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Descrição:
   ----------
    layout final aprovado:
     - ZONA HARDWARE expandida com vizinhos (linha 2 de 3)
     - ZONA GRÁFICOS removida (lv_chart eliminado)
     - ZONA RADAR adicionada (lv_canvas, HLK-LD2450 X/Y mm)
     - API nova: display_manager_set_radar()
     - Todas as APIs anteriores mantidas com assinaturas iguais

   Pré-requisito de inicialização (ordem obrigatória):
   ----------------------------------------------------
     1. nvs_flash_init()
     2. post_config_init()
     3. display_manager_init()

   Configuração lv_conf.h necessária:
   ------------------------------------
     #define LV_USE_CANVAS          1   ← OBRIGATÓRIO v5.0
     #define LV_USE_BAR             1
     #define LV_USE_LABEL           1
     #define LV_FONT_MONTSERRAT_10  1
     #define LV_FONT_MONTSERRAT_12  1
     #define LV_FONT_MONTSERRAT_14  1
     #define LV_FONT_MONTSERRAT_18  1
     (LV_USE_CHART já não é necessário)

   Dependências directas:
   ----------------------
     st7789.h        : driver SPI do display
     system_config.h : LCD_H_RES, LCD_V_RES, LIGHT_MIN
     hw_config.h     : LCD_PIN_*
     post_config.h   : post_get_name(), post_get_id()
     lvgl            : v8.3.x

   Módulos que chamam este:
   ------------------------
     state_machine.c  → set_status(), set_leader()
     wifi_manager.c   → set_wifi()
     comm_manager.c   → set_neighbors()
     radar_manager.c  → set_traffic(), set_speed(),
                        set_hardware(), set_radar()
     app_main.c       → init(), tick(), task()

============================================================ */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   CONSTANTES PÚBLICAS
============================================================ */

/** Número máximo de objectos simultâneos no radar */
#define RADAR_MAX_OBJ       3

/** Comprimento máximo do rasto de cada objecto (frames) */
#define RADAR_TRAIL_MAX     8

/**
 * Alcance frontal máximo do HLK-LD2450 em milímetros.
 * Corresponde a RADAR_DETECT_M × 1000 (7m = 7000mm).
 * Usado pelo main.c para inicializar posições Y dos objectos
 * simulados e pelo display_manager.c para escalar o canvas.
 * Deve coincidir com RADAR_DETECT_M × 1000 em system_config.h.
 */
#define RADAR_MAX_MM        7000

/**
 * Metade do alcance lateral do sensor em milímetros.
 * O campo de visão X vai de -RADAR_XSPAN_MM a +RADAR_XSPAN_MM.
 * Negativo = esquerda do sensor; positivo = direita.
 */
#define RADAR_XSPAN_MM      5000


/* ============================================================
   ESTRUTURA DE OBJECTO DO RADAR
   Preenchida pelo radar_manager a partir do frame UART
   do HLK-LD2450 e passada a display_manager_set_radar().

   Coordenadas no sistema do sensor:
     x_mm : posição lateral em mm
            negativo = esquerda do sensor
            positivo = direita do sensor
     y_mm : distância frontal em mm
            0    = mesmo ponto que o sensor
            7000 = limite do alcance (RADAR_DETECT_M × 1000)
============================================================ */
typedef struct {
    int   x_mm;                         /* Posição lateral (mm)    */
    int   y_mm;                         /* Distância frontal (mm)  */
    int   trail_x[RADAR_TRAIL_MAX];     /* Rasto — posições X (mm) */
    int   trail_y[RADAR_TRAIL_MAX];     /* Rasto — posições Y (mm) */
    uint8_t trail_len;                  /* Entradas válidas no rasto*/
} radar_obj_t;


/* ============================================================
   FUNÇÕES DE CICLO DE VIDA
============================================================ */

/**
 * @brief  Inicializa ST7789, LVGL e interface v5.0.
 *         Chamar UMA VEZ em app_main(), depois de
 *         nvs_flash_init() e post_config_init().
 */
void display_manager_init(void);

/**
 * @brief  Incrementa ticker LVGL. Chamar a cada 1 ms.
 * @param  ms  Milissegundos decorridos.
 */
void display_manager_tick(uint32_t ms);

/**
 * @brief  Processa eventos LVGL. Chamar a cada 5-10 ms.
 */
void display_manager_task(void);


/* ============================================================
   FUNÇÕES DE ACTUALIZAÇÃO DE ESTADO
   Assinaturas idênticas às versões anteriores (v3.0..v4.1).
============================================================ */

/**
 * @brief  Actualiza badge de modo FSM (zona identidade).
 *         Estados: "IDLE", "LIGHT ON", "SAFE MODE",
 *                  "MASTER", "AUTONOMO".
 * @param  status  String do estado (não pode ser NULL).
 */
void display_manager_set_status(const char *status);

/**
 * @brief  Atalho: true → "MASTER" verde; false → "IDLE".
 * @param  is_leader  Estado de liderança.
 */
void display_manager_set_leader(bool is_leader);

/**
 * @brief  Actualiza WiFi e IP (linha 1 zona hardware).
 * @param  connected  true se ligado.
 * @param  ip         String do IP (pode ser NULL).
 */
void display_manager_set_wifi(bool connected, const char *ip);

/**
 * @brief  Actualiza estado radar e brilho DALI
 *         (linhas 1 e 3 zona hardware).
 * @param  radar_ok    true se radar responde.
 * @param  brightness  Brilho DALI 0-100 %.
 */
void display_manager_set_hardware(bool radar_ok, uint8_t brightness);

/**
 * @brief  Actualiza cards T e Tc (zona tráfego).
 *         T=0 → cinzento; T>0 → amarelo/ciano.
 * @param  T   Veículos detectados localmente (0-999).
 * @param  Tc  Veículos anunciados via UDP (0-999).
 */
void display_manager_set_traffic(int T, int Tc);

/**
 * @brief  Actualiza card km/h (zona tráfego).
 *         speed=0 → cinzento; speed>0 → violeta.
 * @param  speed  Velocidade em km/h (0-300).
 */
void display_manager_set_speed(int speed);

/**
 * @brief  Actualiza vizinhos (linha 2 zona hardware).
 *         IP completo + estado OK/OFF.
 *         Online → verde; offline → vermelho.
 * @param  nebL     IP vizinho esquerdo (NULL → "---").
 * @param  nebR     IP vizinho direito  (NULL → "---").
 * @param  leftOk   true se esquerdo responde.
 * @param  rightOk  true se direito responde.
 */
void display_manager_set_neighbors(const char *nebL,
                                   const char *nebR,
                                   bool        leftOk,
                                   bool        rightOk);

/**
 * @brief  Actualiza o radar com os objectos detectados.
 *         API NOVA v5.0 — chamar a cada frame do HLK-LD2450.
 *
 *         O radar_manager preenche o array radar_obj_t com
 *         as coordenadas X/Y em mm extraídas do frame UART
 *         (byte[2]=distância, byte[3]=velocidade) e chama
 *         esta função para redesenhar o canvas.
 *
 *         Se count=0, o radar fica vazio (sem pontos
 *         vermelhos) — T deve também ser 0 neste caso.
 *
 * @param  objs   Array de radar_obj_t (pode ser NULL se count=0).
 * @param  count  Número de objectos (0..RADAR_MAX_OBJ).
 */
void display_manager_set_radar(const radar_obj_t *objs,
                               uint8_t            count);

#endif /* DISPLAY_MANAGER_H */