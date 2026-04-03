================================================================================
MODULO: config
VERSAO: 2.2 | DATA: 2026-03-25
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Modulo central de configuracao do sistema. Contem tres ficheiros:

  hw_config.h     -- pinos GPIO e perifericos de hardware (nunca editar)
  system_config.h -- parametros globais do sistema (EDITAR POR POSTE)
  post_config.h/c -- ID e Nome persistentes em flash NVS

O QUE FAZ
---------
  hw_config.h:
    Define todos os pinos GPIO do ESP32 para os perifericos:
    Display ST7789, Radar HLK-LD2450 (UART2), Luminaria PWM.
    Nao deve ser editado -- os pinos sao fixos por hardware.

  system_config.h:
    UNICO FICHEIRO A EDITAR ao flashar cada poste.
    Define identidade, rede Wi-Fi, protocolo UDP, fisica da instalacao,
    modo do radar, niveis de iluminacao e timings do sistema.

  post_config.c / post_config.h:
    Gestao da configuracao persistente em flash NVS.
    No primeiro arranque popula NVS com valores de system_config.h.
    Em arranques subsequentes carrega da NVS -- sobrevive a reboots.

COMO CONFIGURAR UM NOVO POSTE
------------------------------
  Editar APENAS estas 3 linhas em system_config.h:
    #define POSTE_ID        2        <- ID unico por poste
    #define POSTE_NAME      "POSTE 02"  <- nome legivel
    #define POST_POSITION   1        <- posicao na cadeia (0=inicio)
  Compilar e flashar. NVS populada automaticamente no primeiro boot.

API PUBLICA (post_config.h)
----------------------------
  void        post_config_init(void)   -- carrega NVS ou usa defaults
  uint8_t     post_get_id(void)        -- retorna ID actual
  const char *post_get_name(void)      -- retorna nome actual
  void        post_set_id(uint8_t id)  -- altera ID e persiste
  void        post_set_name(char *name)-- altera nome e persiste

PINOS DE HARDWARE (hw_config.h)
---------------------------------
  DISPLAY ST7789 (SPI2_HOST):
    GPIO 23 -- MOSI (SDA/dados)
    GPIO 18 -- SCLK (SCL/clock)
    GPIO  5 -- CS (Chip Select)
    GPIO 32 -- DC (Data/Command)
    GPIO 33 -- RST (Reset hardware)
    GPIO 25 -- BL (Backlight)
    Alimentacao: 3.3V

  RADAR HLK-LD2450 (UART2):
    GPIO 27 -- TX do ESP32 --> RX do Radar
    GPIO 22 -- RX do ESP32 <-- TX do Radar
    Baud: 115200 (auto-detect: 256000/115200/57600)
    Alimentacao: 5V (nao 3.3V!)

  LUMINARIA PWM (LEDC):
    GPIO 26 -- sinal PWM 5000 Hz para dimmer/DALI
    Frequencia: 5000 Hz | Resolucao: 8 bits (0-255)

PARAMETROS SYSTEM_CONFIG.H
-----------------------------
  Identidade  : POSTE_ID, POSTE_NAME, POST_POSITION
  Wi-Fi       : WIFI_SSID, WIFI_PASS, WIFI_RETRY_ATTEMPTS(5)
  UDP         : UDP_PORT(5005), MAX_NEIGHBORS(4), NEIGHBOR_TIMEOUT_MS(8000)
  Fisica      : POSTE_DIST_M(50), RADAR_DETECT_M(7), RADAR_MAX_M(10)
  Radar       : USE_RADAR (0=simulado, 1=hardware HLK-LD2450)
  Iluminacao  : LIGHT_MIN(10%), LIGHT_MAX(100%), LIGHT_SAFE_MODE(50%)
  Timing      : TRAFIC_TIMEOUT_MS(5000), DETECTION_TIMEOUT_MS(1000)
  Display     : LCD_H_RES(240), LCD_V_RES(240), DISPLAY_UPDATE_MS(500)

DEPENDENCIAS
------------
  nvs_flash.h  -- inicializacao da flash NVS
  nvs.h        -- leitura/escrita de chaves NVS
  esp_log.h    -- logging

  NOTA: nvs_flash_init() deve ser chamado em app_main()
        ANTES de post_config_init().

REFERENCIAS
-----------
  ESP-IDF NVS (Non-Volatile Storage):
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html

  ESP32 GPIO Reference:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html

  ESP32 Pinout (mapa visual dos pinos):
    https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

  YouTube - ESP32 NVS Storage (Random Nerd Tutorials):
    https://www.youtube.com/@RandomNerdTutorials

  YouTube - Espressif Official Channel:
    https://www.youtube.com/@EspressifSystems
================================================================================
