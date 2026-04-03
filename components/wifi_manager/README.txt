================================================================================
MODULO: wifi_manager
VERSAO: 1.5 | DATA: 2026-03-24
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Gere a ligacao Wi-Fi do ESP32 em modo STA (Station) com DHCP.
Trata eventos de ligacao/desligacao, implementa reconexao automatica
em background com intervalo configuravel, e notifica o display_manager
sempre que o estado de rede muda.

O QUE FAZ
---------
  - Inicializa Wi-Fi em modo STA com as credenciais de system_config.h
  - Liga automaticamente ao arranque
  - Trata WIFI_EVENT_STA_DISCONNECTED: retry ate WIFI_RETRY_ATTEMPTS
  - Apos esgotar retries: aguarda WIFI_RECONNECT_MS e tenta de novo
  - Ao obter IP: notifica display_manager_set_wifi(true, ip)
  - Ao desligar: notifica display_manager_set_wifi(false, ...)
  - Sem chamadas bloqueantes -- tudo por eventos FreeRTOS

API PUBLICA
-----------
  void        wifi_manager_init(void)        -- inicia Wi-Fi STA
  bool        wifi_manager_is_connected(void) -- true se IP obtido
  const char *wifi_manager_get_ip(void)      -- "x.x.x.x" ou ""
  void        wifi_manager_reset_retry(void) -- reset contador de retries

EVENTOS TRATADOS
-----------------
  WIFI_EVENT_STA_START         -> esp_wifi_connect()
  WIFI_EVENT_STA_CONNECTED     -> ligado (aguarda IP)
  WIFI_EVENT_STA_DISCONNECTED  -> retry ou aguarda RECONNECT_MS
  IP_EVENT_STA_GOT_IP          -> guarda IP, notifica display

PARAMETROS (system_config.h)
------------------------------
  WIFI_SSID              "wifi"    -- nome da rede
  WIFI_PASS              "****"    -- password
  WIFI_RETRY_ATTEMPTS    5         -- tentativas antes de pausar
  WIFI_RECONNECT_MS      30000     -- pausa entre series de tentativas (30s)

DEPENDENCIAS
------------
  display_manager.h  -- display_manager_set_wifi() (notificacao visual)
  system_config.h    -- WIFI_SSID, WIFI_PASS, WIFI_RETRY_ATTEMPTS
  esp_wifi.h         -- API Wi-Fi do ESP-IDF
  esp_event.h        -- sistema de eventos ESP-IDF
  esp_netif.h        -- interface de rede (DHCP)
  esp_log.h          -- logging

  NOTA ARQUITECTURAL:
    wifi_manager.c -> display_manager.h (so nesta direccao)
    display_manager.h NAO inclui wifi_manager.h (evita dependencia circular)

ORDEM DE INICIALIZACAO (obrigatoria)
--------------------------------------
  esp_netif_init()                     -- antes de wifi_manager_init()
  esp_event_loop_create_default()      -- antes de wifi_manager_init()
  wifi_manager_init()                  -- depois dos dois acima

NOTAS
-----
  O comm_manager (UDP) so deve ser iniciado depois de
  wifi_manager_is_connected() retornar true e IP obtido.
  O main.c verifica isto no loop principal:
    if (!comm_iniciado && wifi_manager_is_connected()) comm_init();

REFERENCIAS
-----------
  ESP-IDF Wi-Fi Driver:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html

  ESP-IDF Wi-Fi Getting Started:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html

  ESP-IDF exemplo Wi-Fi Station:
    https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station

  YouTube - ESP32 Wi-Fi STA ESP-IDF (Espressif):
    https://www.youtube.com/@EspressifSystems

  YouTube - ESP32 Wi-Fi Tutorial (Random Nerd Tutorials):
    https://www.youtube.com/@RandomNerdTutorials
================================================================================
