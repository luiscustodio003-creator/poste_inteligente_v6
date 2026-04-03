================================================================================
MODULO: comm_manager
VERSAO: 2.0 | DATA: 2026-03-20
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Camada de abstracao de alto nivel sobre o udp_manager.
Gere o posicionamento relativo dos vizinhos (esquerdo/direito),
calcula automaticamente o ETA (tempo estimado de chegada do veiculo
ao proximo poste) e implementa a logica de lideranca MASTER/SLAVE.

O QUE FAZ
---------
  - Determina vizinho esquerdo (posicao = POST_POSITION - 1)
    e vizinho direito (posicao = POST_POSITION + 1)
  - Calcula ETA: (POSTE_DIST_M - RADAR_DETECT_M) / vel_m_s * 1000 ms
  - Decide quem e MASTER:
      POST_POSITION == 0  -> sempre MASTER
      Vizinho esq. offline -> promovido a MASTER temporariamente
  - Envia TC_INC + SPD ao vizinho direito ao detectar veiculo
  - Envia PASSED ao vizinho esquerdo quando veiculo chega
  - Propaga MASTER_CLAIM em cadeia quando pos=0 volta online
  - comm_status_ok(): false se vizinho esq. e dir. ambos offline

CALCULO DO ETA
---------------
  ETA_ms = (POSTE_DIST_M - RADAR_DETECT_M) / (vel_kmh / 3.6) * 1000

  Exemplo: vel=50 km/h, POSTE_DIST_M=50m, RADAR_DETECT_M=7m
    ETA = (50 - 7) / (50/3.6) * 1000
        = 43 / 13.89 * 1000
        = ~3094 ms

  O poste seguinte recebe o ETA e acende MARGEM_ACENDER_MS (500ms)
  antes do veiculo chegar, garantindo luz a 100% na chegada.

API PUBLICA
-----------
  bool       comm_init(void)
  void       comm_discover(void)
  bool       comm_send_spd(float speed)
  bool       comm_send_tc_inc(float speed)
  bool       comm_notify_prev_passed(float speed)
  void       comm_send_master_claim(void)
  void       comm_send_status(neighbor_status_t status)
  neighbor_t *comm_get_neighbor_left(void)
  neighbor_t *comm_get_neighbor_right(void)
  size_t     comm_get_neighbors(neighbor_t *list, size_t max)
  bool       comm_is_master(void)
  bool       comm_status_ok(void)
  bool       comm_left_online(void)
  bool       comm_right_online(void)

DEPENDENCIAS
------------
  udp_manager.h   -- envio/recepcao UDP e tabela de vizinhos (neighbor_t)
  wifi_manager.h  -- verificar se rede esta activa (comm_status_ok)
  system_config.h -- POST_POSITION, POSTE_DIST_M, RADAR_DETECT_M

PARAMETROS CONFIGURÁVEIS (system_config.h)
-------------------------------------------
  POST_POSITION    1    Posicao na cadeia (0=inicio/MASTER)
  POSTE_DIST_M    50    Distancia entre postes em metros
  RADAR_DETECT_M   7    Distancia de deteccao do radar em metros

NOTAS
-----
  comm_init() deve ser chamado APENAS apos Wi-Fi com IP obtido.
  O main.c verifica: if (!comm_iniciado && wifi_is_connected()) comm_init()

  comm_is_master() e chamado pela state_machine a cada ciclo (100ms)
  para actualizar o estado MASTER/SLAVE dinamicamente.

REFERENCIAS
-----------
  ESP-IDF Sockets / lwIP:
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/lwip.html

  FreeRTOS Tasks e sincronizacao:
    https://www.freertos.org/taskandcr.html

  GitHub Espressif ESP-IDF:
    https://github.com/espressif/esp-idf

  YouTube - ESP32 Networking (Espressif Official):
    https://www.youtube.com/@EspressifSystems
================================================================================
