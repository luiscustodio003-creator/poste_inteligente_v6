================================================================================
MODULO: display_manager
VERSAO: 5.3 | DATA: 2026-03-25
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Camada de apresentacao LVGL que gere toda a interface grafica do poste.
Implementa um layout de 4 zonas num ecra ST7789 240x240, com actualizacao
thread-safe via fila de mensagens FreeRTOS (sem acesso directo ao LVGL
fora da main_task, evitando deadlocks e Task Watchdog).

O QUE FAZ
---------
  - Inicializa LVGL + driver ST7789 + buffer de render duplo
  - Cria e gere todos os widgets da interface
  - Recebe actualizacoes das outras tasks via xQueueSend() (non-blocking)
  - Aplica cada mensagem ao LVGL exclusivamente na main_task
  - Renderiza canvas do radar em tempo real com pontos, rastos e sweep

LAYOUT DO ECRA (240x240 px)
-----------------------------
  y=  0..35  ZONA IDENTIDADE  : nome do poste, badge FSM (IDLE/MASTER/etc)
  y= 36..36  Separador
  y= 37..92  ZONA HARDWARE    : WiFi+IP, Radar (REAL/SIM/FAIL), vizinhos, DALI
  y= 93..93  Separador
  y= 94..145 ZONA TRAFEGO     : cards T (aqui) | Tc (a vir) | km/h
  y=146..146 Separador
  y=147..239 ZONA RADAR       : canvas 230x90 px HLK-LD2450 X/Y com rasto

CORES DO DISPLAY
-----------------
  REAL (radar ok)    -> Radar: REAL   (verde  #22C55E)
  SIM  (simulado)    -> Radar: SIM    (amarelo #F5C542)
  FAIL (sem resposta)-> Radar: FAIL   (vermelho #FF3333)
  WiFi ligado        -> WiFi: ON      (verde)
  WiFi desligado     -> WiFi: OFF     (vermelho)
  Vizinho online     -> IP             (verde)
  Vizinho offline    -> IP             (vermelho)
  Card T activo      -> valor          (amarelo dourado)
  Card Tc activo     -> valor          (ciano)
  Card km/h activo   -> valor          (violeta)

CANVAS DO RADAR
----------------
  - Fundo preto-esverdeado com arcos de alcance (2m, 5m, 8m)
  - Sweep animado (sector verde, 3 graus por frame, 100ms)
  - Cada alvo: ponto branco (halo) + rasto de 8 posicoes anteriores
  - Eixo X: posicao lateral em mm (esq/dir do sensor)
  - Eixo Y: distancia ao sensor em mm (topo=longe, base=perto)
  - Retencao de 500ms: ponto mantido 500ms sem frame (anti-flicker)

THREAD SAFETY (v5.3)
---------------------
  PROBLEMA: LVGL nao e thread-safe. Acesso concorrente ao heap do LVGL
  (lv_tlsf_malloc/free) causava Task Watchdog e crashes.

  SOLUCAO -- fila de mensagens:
    set_*() -- NON-BLOCKING, apenas xQueueSend() com struct dm_msg_t
    display_manager_task() -- drena a fila e aplica ao LVGL (so main_task)
    display_manager_tick() -- chama lv_tick_inc() (thread-safe por si)

API PUBLICA
-----------
  void display_manager_init(void)
  void display_manager_tick(uint32_t ms)
  void display_manager_task(void)
  void display_manager_set_status(const char *status)
  void display_manager_set_leader(bool is_leader)
  void display_manager_set_wifi(bool connected, const char *ip)
  void display_manager_set_hardware(const char *radar_st, bool ok, uint8_t brightness)
  void display_manager_set_traffic(int T, int Tc)
  void display_manager_set_speed(int speed)
  void display_manager_set_neighbors(nebL, nebR, leftOk, rightOk)
  void display_manager_set_radar(const radar_obj_t *objs, uint8_t count)

ORDEM DE INICIALIZACAO (obrigatoria)
--------------------------------------
  1. nvs_flash_init()
  2. post_config_init()
  3. display_manager_init()   <-- so depois dos dois acima

DEPENDENCIAS
------------
  display_manager.h  -- API publica (inclui radar_manager.h)
  radar_manager.h    -- radar_obj_t (fonte unica de verdade)
  st7789.h           -- driver SPI do display fisico
  system_config.h    -- LCD_H_RES, LCD_V_RES, LIGHT_MIN, RADAR_MAX_MM
  hw_config.h        -- LCD_PIN_*
  post_config.h      -- post_get_name(), post_get_id()
  lvgl               -- v8.3.x (LV_USE_CANVAS=1 em lv_conf.h)
  freertos/queue.h   -- fila de mensagens thread-safe
  esp_timer.h        -- timestamp para retencao do ponto radar

CONFIGURACAO LV_CONF.H (obrigatoria)
--------------------------------------
  #define LV_COLOR_DEPTH         16
  #define LV_USE_CANVAS           1
  #define LV_USE_BAR              1
  #define LV_USE_LABEL            1
  #define LV_FONT_MONTSERRAT_10   1
  #define LV_FONT_MONTSERRAT_12   1
  #define LV_FONT_MONTSERRAT_14   1
  #define LV_FONT_MONTSERRAT_18   1

REFERENCIAS
-----------
  LVGL Documentacao oficial (v8.3):
    https://docs.lvgl.io/8.3/

  LVGL GitHub:
    https://github.com/lvgl/lvgl

  LVGL Canvas API:
    https://docs.lvgl.io/8.3/widgets/extra/canvas.html

  LVGL com ESP32 ESP-IDF (getting started):
    https://docs.lvgl.io/8.3/get-started/platforms/espressif.html

  YouTube - LVGL ESP32 Tutorial completo:
    https://www.youtube.com/@EspressifSystems

  YouTube - LVGL GUI Tutorial (Mose Embedded):
    https://www.youtube.com/@MoseEmbedded

  GitHub - ESP-IDF LVGL exemplo oficial:
    https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd
================================================================================
