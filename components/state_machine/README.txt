================================================================================
MODULO: state_machine
VERSAO: 2.9 | DATA: 2026-03-25
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Maquina de estados finita (FSM) que e o cerebro do poste inteligente.
Controla a luminaria com base em deteccoes do radar local, mensagens UDP
dos postes vizinhos, e estado da comunicacao de rede. Implementa logica
de coordenacao em cadeia, fail-safe, e simulador fisico de veiculos.

ESTADOS DO SISTEMA
-------------------
  IDLE       -- T=0, Tc=0 | luminaria a LIGHT_MIN (10%)
  LIGHT_ON   -- T>0 ou Tc>0 | luminaria a LIGHT_MAX (100%)
  SAFE_MODE  -- radar falhou | luminaria a LIGHT_SAFE_MODE (50%) fixo
  MASTER     -- pos=0 ou vizinho esq. offline | gera veiculos simulados
  AUTONOMO   -- UDP falhou | opera so com radar local

CONTADORES T e Tc
------------------
  T  (local)  -- veiculos detectados pelo radar DESTE poste
                 T++ quando radar detecta | T-- quando carro passou (PASSED)
  Tc (caminho)-- veiculos anunciados via UDP a caminho deste poste
                 Tc++ quando recebe TC_INC | Tc-- quando carro chega (RADAR)

  Luz 100% quando T>0 OU Tc>0
  Luz 10%  quando T=0 E  Tc=0 (apos TRAFIC_TIMEOUT_MS sem deteccao)

FLUXO CORRECTO EM CADEIA (A -> B -> C)
----------------------------------------
  1. A detecta carro a X km/h:
       T_A++, envia TC_INC+SPD para B
  2. B recebe TC_INC: Tc_B++, agenda acendimento pre-ETA
     B recebe SPD: calcula ETA, acende (ETA - 500ms) antes do carro
     B propaga TC_INC+SPD para C
  3. Carro chega a B: T_B++, Tc_B--
     B envia PASSED para A (T_A--)
     B envia TC_INC+SPD para C (confirma)
  4. Carro chega a C: T_C++, Tc_C--
     C envia PASSED para B (T_B--)

PRE-ACENDIMENTO (anti-escuridao)
----------------------------------
  Quando recebe SPD(vel, eta_ms):
    acender_em = agora + eta_ms - MARGEM_ACENDER_MS (500ms)
  O poste acende MARGEM_ACENDER_MS antes do carro chegar.
  Se ETA <= MARGEM: acende imediatamente.
  Fallback: se TC_INC chegar sem SPD por >1s, acende por precaucao.

DEBOUNCE DO RADAR (v2.7)
--------------------------
  sm_on_radar_detect() so e chamado na transicao ausente->presente
  (rising edge). Sem debounce, chamaria a cada 100ms com veiculo
  no campo, enviando TC_INC+PASSED repetidamente.

GESTAO DE FALHAS
-----------------
  Radar falha (>10 leituras sem frame):
    -> SAFE_MODE (50%), envia STATUS:SAFE aos vizinhos
  Radar recupera (3 leituras OK):
    -> sai de SAFE_MODE, volta ao estado anterior
  Vizinho direito offline:
    -> Tc=0 imediato (nao ha ninguem para receber o carro)
  Vizinho esquerdo offline:
    -> promovido a MASTER (gera veiculos simulados se USE_RADAR=0)
    -> T preso: se vizinho esq. offline >T_STUCK_TIMEOUT_MS, forca T=0
  Vizinho esquerdo volta online:
    -> cede MASTER, propaga MASTER_CLAIM em cadeia
  UDP falha:
    -> AUTONOMO (so radar local, sem coordenacao)
  UDP recupera:
    -> volta a IDLE/MASTER conforme posicao

SIMULADOR FISICO (USE_RADAR=0)
--------------------------------
  So o poste MASTER gera veiculos automaticamente (rotacao 30/50/80/50 km/h).
  Os postes IDLE recebem o veiculo via UDP (TC_INC) e activam o simulador
  local para mostrar o ponto no canvas quando o veiculo entra no raio.
  Estados: AGUARDA -> ENTRAR -> EM_VIA -> DETECTADO -> SAIU -> AGUARDA

API PUBLICA
-----------
  void           state_machine_init(void)
  void           state_machine_update(bool comm_ok, bool is_master)
  void           sm_on_radar_detect(float vel)
  void           sm_on_right_neighbor_offline(void)
  void           sm_on_right_neighbor_online(void)
  void           sm_inject_test_car(float vel)
  system_state_t state_machine_get_state(void)
  const char    *state_machine_get_state_name(void)
  int            state_machine_get_T(void)
  int            state_machine_get_Tc(void)
  float          state_machine_get_last_speed(void)
  bool           state_machine_radar_ok(void)
  -- Apenas USE_RADAR=0 --
  bool           sim_get_objeto(float *x_mm, float *y_mm)
  void           sim_notificar_chegada(float vel_kmh)

CALLBACKS UDP (weak -- implementados aqui, declarados em udp_manager.h)
------------------------------------------------------------------------
  void on_tc_inc_received(float speed)
  void on_prev_passed_received(void)
  void on_spd_received(float speed, uint32_t eta_ms)
  void on_master_claim_received(int from_id)

TIMINGS CRITICOS (system_config.h)
------------------------------------
  TRAFIC_TIMEOUT_MS     5000  -- tempo apos ultimo carro para baixar luz
  DETECTION_TIMEOUT_MS  1000  -- timeout de deteccao activa
  T_STUCK_TIMEOUT_MS   15000  -- (3x TRAFIC) -- T preso com viz. esq. offline
  TC_TIMEOUT_MS        10000  -- (2x TRAFIC) -- Tc sem carro chegar
  MARGEM_ACENDER_MS      500  -- antecipacao de acendimento pre-ETA
  RADAR_FAIL_COUNT        10  -- leituras sem frame -> SAFE_MODE
  RADAR_OK_COUNT           3  -- leituras OK -> sair de SAFE_MODE

DEPENDENCIAS
------------
  dali_manager.h   -- controlo de brilho PWM
  comm_manager.h   -- envio de mensagens UDP
  radar_manager.h  -- leitura do sensor / simulador
  system_config.h  -- todas as constantes de timing e fisica
  esp_timer.h      -- timestamps de alta resolucao
  esp_log.h        -- logging

REFERENCIAS
-----------
  FreeRTOS -- conceito de maquina de estados em RTOS:
    https://www.freertos.org/

  Teoria FSM (Finite State Machine) embebidos:
    https://www.embedded.com/state-machine-design-in-c/

  ESP-IDF esp_timer (timestamps microsegundos):
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html

  YouTube - State Machines em C Embebido:
    Pesquisar "embedded state machine C" no YouTube

  YouTube - ESP32 FreeRTOS Tasks e Sincronizacao:
    https://www.youtube.com/@EspressifSystems
================================================================================
