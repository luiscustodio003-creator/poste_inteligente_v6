================================================================================
MODULO: config
================================================================================
Projecto  : Poste Inteligente
Estudantes: Luis Custodio | Tiago Moreno
Plataforma: ESP32 (ESP-IDF)

DESCRICAO
---------
Modulo central de configuracao do sistema. Contem dois ficheiros:

  system_config.h
    Ficheiro de parametros globais. E o UNICO ficheiro que deve ser editado
    ao flashar cada poste. Define identificacao, rede Wi-Fi, protocolo UDP,
    fisica da instalacao (distancias), radar, iluminacao e display.

  post_config.c / post_config.h
    Gestao da configuracao persistente do poste na flash NVS do ESP32.
    No primeiro arranque apos flash, popula a NVS com os valores de
    POSTE_ID e POSTE_NAME definidos em system_config.h. Em arranques
    subsequentes carrega da NVS, garantindo que o nome e ID sobrevivem
    a cortes de energia e rearranques.

FUNCIONALIDADES
---------------
  system_config.h:
    - POSTE_ID, POSTE_NAME, POST_POSITION : identificacao unica por poste
    - WIFI_SSID/PASS, WIFI_RETRY_ATTEMPTS : credenciais Wi-Fi STA
    - WIFI_RECONNECT_MS                   : intervalo reconexao background
    - NETWORK_SSID/PASS/IP/BROADCAST      : AP proprio (fallback STA)
    - UDP_PORT, MAX_NEIGHBORS             : protocolo UDP
    - POSTE_DIST_M (50m), RADAR_DETECT_M (7m): fisica da instalacao
    - USE_RADAR (0=simulado, 1=hardware)  : modo do radar
    - LIGHT_MIN/MAX/SAFE_MODE             : niveis de brilho (%)
    - TRAFIC_TIMEOUT_MS (5000ms)          : tempo apos ultimo carro
    - DISPLAY_UPDATE_MS (500ms)           : intervalo actualizacao display

  post_config.c:
    - post_config_init() : carrega NVS ou popula com defaults
    - post_get_id()      : retorna ID actual do poste
    - post_get_name()    : retorna nome actual do poste
    - post_set_id()      : define novo ID e persiste na NVS
    - post_set_name()    : define novo nome e persiste na NVS

COMO CONFIGURAR UM NOVO POSTE
------------------------------
  1. Abrir components/config/system_config.h
  2. Editar as 3 linhas marcadas [EDITAR]:
       #define POSTE_ID       2
       #define POSTE_NAME     "POSTE 02"
       #define POST_POSITION  1
  3. Compilar e flashar
  4. No primeiro arranque a NVS e populada automaticamente

DEPENDENCIAS
------------
  - nvs_flash  : armazenamento persistente na flash NVS do ESP32
                 https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html
  - esp_log    : sistema de logging do ESP-IDF
                 https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/log.html

NOTA
----
  nvs_flash_init() deve ser chamado em app_main() ANTES de post_config_init().
  Este modulo nao inicializa a NVS -- assume que ja esta pronta.
================================================================================