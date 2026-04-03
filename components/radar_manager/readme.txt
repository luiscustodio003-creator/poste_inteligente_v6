================================================================================
MODULO: radar_manager
VERSAO: 2.7 | DATA: 2026-03-30
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Driver para o sensor de radar HLK-LD2450 (24 GHz FMCW) via UART2 do ESP32.
Descodifica frames binarios de 30 bytes, extrai posicao (X/Y mm) e velocidade
(km/h) de ate 3 alvos simultaneos, e mantem historico de rasto para o canvas
do display. Inclui modo simulado (USE_RADAR=0) e auto-deteccao de baud rate.

O QUE FAZ
---------
  - Inicializa UART2 com baud rate configuravel (115200 por omissao)
  - Auto-detecta baud rate: testa 256000, 115200, 57600 (300ms cada)
  - Le e acumula bytes num ring buffer circular de 256 bytes
  - Procura frame valido: header AA FF 03 00 + footer 55 CC (30 bytes)
  - Descodifica ate 3 alvos por frame:
      X mm   : posicao lateral (negativo=esquerda, positivo=direita)
      Y mm   : distancia ao sensor (sempre positivo)
      vel    : velocidade em cm/s convertida para km/h
      dist   : distancia euclidiana em mm (campo uint16 do frame)
  - Mantem rasto de 8 posicoes anteriores por alvo (para o canvas)
  - Cache s_last_data: get_objects() usa o ultimo frame lido (sem dupla leitura)
  - radar_is_connected(): false apos 20 ciclos consecutivos sem frame valido
  - Modo simulado: aceita radar_simulated_input_t injectado pela FSM

FORMATO DO FRAME HLK-LD2450 (30 bytes)
----------------------------------------
  Byte  0- 3: Header     AA FF 03 00
  Byte  4-11: Alvo 1     X(2) Y(2) Vel(2) Dist(2)
  Byte 12-19: Alvo 2     X(2) Y(2) Vel(2) Dist(2)
  Byte 20-27: Alvo 3     X(2) Y(2) Vel(2) Dist(2)
  Byte 28-29: Footer     55 CC

  Decodificacao de sinal (Manual pag. 13):
    Bit 15 (MSB) = 1 -> Positivo
    Bit 15 (MSB) = 0 -> Negativo
    Magnitude = 15 bits inferiores

  Velocidade: valor em cm/s -> dividir por 100 * 3.6 = km/h

AUTO-DETECCAO DE BAUD RATE
----------------------------
  Alguns HLK-LD2450 saem de fabrica a 256000 baud (nao 115200).
  radar_auto_detect_baud() testa em sequencia:
    1. 256000 baud -- aguarda 300ms, procura frame valido
    2. 115200 baud -- aguarda 300ms, procura frame valido
    3.  57600 baud -- aguarda 300ms, procura frame valido
  Se encontrar frame valido, configura UART para esse baud e retorna-o.
  Se falhar todos, volta a 115200 (fallback seguro) e retorna 0.

ESTADO DO RADAR NO DISPLAY
----------------------------
  radar_get_status_str() retorna:
    "REAL" -- UART a receber frames validos   (label verde)
    "SIM"  -- modo simulado (USE_RADAR=0)     (label amarelo)
    "FAIL" -- UART sem frames ha >2 segundos  (label vermelho)

API PUBLICA
-----------
  void         radar_init(radar_mode_t mode)
  void         radar_flush_rx(void)              -- limpa backlog UART
  int          radar_auto_detect_baud(void)       -- retorna baud detectado
  bool         radar_read_data(out_data, sim_input)
  uint8_t      radar_manager_get_objects(objs, max) -- usa cache
  bool         radar_is_connected(void)
  const char  *radar_get_status_str(void)         -- "REAL"/"SIM"/"FAIL"
  radar_mode_t radar_get_mode(void)
  bool         radar_vehicle_in_range(data)
  float        radar_get_closest_speed(data)

MODOS DE OPERACAO
------------------
  RADAR_MODE_DEFAULT   -- alias de UART (USE_RADAR=1)
  RADAR_MODE_UART      -- hardware real HLK-LD2450
  RADAR_MODE_SIMULATED -- dados injectados pela FSM (USE_RADAR=0)
  RADAR_MODE_NETWORK   -- reservado para uso futuro

PINOS HARDWARE
--------------
  GPIO 22 -- RX do ESP32 <-- TX do radar  (dados do sensor para o ESP32)
  GPIO 27 -- TX do ESP32 --> RX do radar  (comandos, geralmente nao usados)
  UART2, 115200/256000 baud, 8N1
  Alimentacao: 5V (OBRIGATORIO -- nao funciona a 3.3V)

DEPENDENCIAS
------------
  hw_config.h       -- RADAR_UART_PORT, RADAR_PIN_TX/RX, RADAR_BAUD_RATE
  system_config.h   -- USE_RADAR, RADAR_MAX_MM
  driver/uart.h     -- API UART do ESP-IDF
  esp_log.h         -- logging

DATASHEET E MANUAL HLK-LD2450
--------------------------------
  Manual oficial HLK-LD2450 (protocolo, formato de frame, comandos AT):
    https://www.hlktech.net/index.php?id=986

  Datasheet HLK-LD2450 (Hi-Link):
    https://www.hlktech.net/index.php?id=986

  Pagina do produto Hi-Link:
    https://www.hlktech.net/

REFERENCIAS
-----------
  ESP-IDF UART Driver:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html

  HLK-LD2450 no GitHub (projecto de referencia):
    https://github.com/rain931215/ESPHome-HLK-LD2450

  YouTube - HLK-LD2450 radar tutorial:
    Pesquisar "HLK-LD2450 ESP32" no YouTube

  YouTube - UART ESP32 ESP-IDF (Espressif):
    https://www.youtube.com/@EspressifSystems

  24GHz FMCW Radar - conceitos:
    https://www.ti.com/lit/an/swra553a/swra553a.pdf
================================================================================
