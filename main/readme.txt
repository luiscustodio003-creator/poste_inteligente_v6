================================================================================
POSTE INTELIGENTE — SISTEMA EMBARCADO DE ILUMINACAO ADAPTATIVA
Documentacao Tecnica v7.0
================================================================================
Projecto  : Poste Inteligente
Estudantes: Luis Custodio | Tiago Moreno
Plataforma: ESP32 (ESP-IDF)

================================================================================
1. VISAO GERAL
================================================================================
Sistema de iluminacao publica adaptativa baseado em ESP32. Cada poste detecta
veiculos com radar HLK-LD2450 e comunica com os postes vizinhos via UDP/Wi-Fi.
A logica de acendimento e baseada em dois contadores por poste:

  T  = carros presentes (radar confirmou chegada)
  Tc = carros a caminho (UDP informou, ainda nao chegaram)

  Luz 100% quando T>0 OU Tc>0
  Luz 10%  apenas quando T=0 E Tc=0 (apos TRAFIC_TIMEOUT_MS)
  Luz NUNCA vai a 0% -- minimo sempre LIGHT_MIN (10%)

================================================================================
2. ARQUITECTURA
================================================================================
  main.c
    +-- display_manager    (LVGL 8.3, ST7789 240x240)
    +-- state_machine      (FSM: T/Tc, IDLE/LIGHT_ON/SAFE/MASTER/AUTONOMO)
    +-- comm_manager       (abstraccao UDP, ETA, MASTER_CLAIM)
    |     +-- udp_manager  (socket UDP, protocolo completo)
    |     +-- wifi_manager (STA + AP fallback + reconexao background)
    +-- post_config        (ID/nome persistidos na NVS)
    +-- dali_manager       (PWM LEDC, brilho 0-100%)
    +-- radar_manager      (HLK-LD2450 UART ou simulado)
    +-- config             (hw_config.h + system_config.h)

================================================================================
3. PROTOCOLO UDP
================================================================================
  DISCOVER:<id>:<nome>:<pos>            Broadcast periodico (5s)
  DISCOVER_ACK:<id>:<nome>:<pos>:<ip>   Resposta ao DISCOVER
  SPD:<id>:<nome>:<pos>:<vel>:<eta>:<dist>  Velocidade + ETA calculado
  TC_INC:<id>:<pos>:<vel>               Carro a caminho (vel<0 = passou)
  ACK:<id>:<msg_type>                   Confirmacao
  STATUS:<id>:<pos>:<estado>            OK/OFFLINE/SAFE/MASTER
  MASTER_CLAIM:<id>:<pos>               Reclama lideranca (pos=0 prioritario)

================================================================================
4. LOGICA DE DETECCAO E PROPAGACAO
================================================================================
Quando radar do Poste A deteta carro a 7m:
  1. A acende 100% imediatamente
  2. A envia TC_INC para B (Tc[B]++)
  3. A calcula ETA = (50m - 7m) / velocidade
  4. A envia SPD+ETA para B
  5. A notifica A-1 que carro passou (T[A-1]--)
  6. B acende ao receber TC_INC
  7. B repete o processo para C, etc.

Cascata de apagar (T=0 E Tc=0 durante TRAFIC_TIMEOUT_MS):
  A apaga -> informa B -> B verifica T/Tc -> se vazio apaga -> etc.

================================================================================
5. ESTADOS FSM
================================================================================
  STATE_IDLE       T=0 Tc=0   Luz 10%  Repouso normal
  STATE_LIGHT_ON   T>0|Tc>0   Luz 100% Veiculo presente ou a caminho
  STATE_SAFE_MODE  Radar OFF  Luz 50%  Radar falhou
  STATE_MASTER     Lider      Luz var  Controla segmento da linha
  STATE_AUTONOMOUS Sem UDP    Luz var  Radar local assume comando

================================================================================
6. TASKS FREERTOS
================================================================================
  lvgl_task       pri=5  10ms   Rendering display ST7789
  fsm_task        pri=4 100ms   FSM + logica T/Tc + filas
  udp_rx_task     pri=4  20ms   Recepcao UDP (unica a ler socket)
  radar_task      pri=3 100ms   Leitura HLK-LD2450 ou simulado
  discover_task   pri=3   5s    DISCOVER broadcast periodico
  wifi_recon      pri=2  30s    Reconexao STA em background
  update_task     pri=2 500ms   Actualizacao display completo

================================================================================
7. SINCRONIZACAO
================================================================================
  g_radar_event   portMUX spinlock   radar_task <-> fsm_task, update_task
  q_spd           QueueHandle(8)     udp_rx_task -> fsm_task (SPD)
  q_tc            QueueHandle(8)     udp_rx_task -> fsm_task (TC_INC)
  q_master        QueueHandle(4)     udp_rx_task -> fsm_task (MASTER_CLAIM)

================================================================================
8. CONFIGURAR CADA POSTE
================================================================================
Editar APENAS components/config/system_config.h:

  #define POSTE_ID       1          /* ID unico (1, 2, 3...) */
  #define POSTE_NAME     "POSTE 01" /* Nome para display      */
  #define POST_POSITION  0          /* Posicao na cadeia      */
  #define WIFI_SSID      "HotspotX" /* Rede Wi-Fi comum       */
  #define WIFI_PASS      "password" /* Password               */
  #define NETWORK_BROADCAST "192.168.43.255" /* Android hotspot */

Tudo o resto e identico em todos os postes.

================================================================================
9. BROADCAST POR TIPO DE REDE
================================================================================
  Android hotspot : 192.168.43.255
  iPhone hotspot  : 172.20.10.255
  Router caseiro  : 192.168.1.255 ou 192.168.0.255

================================================================================
10. MODULOS E README
================================================================================
  components/config/         README.txt -- configuracao e NVS
  components/wifi_manager/   README.txt -- Wi-Fi STA/AP/background
  components/udp_manager/    README.txt -- protocolo UDP completo
  components/comm_manager/   README.txt -- abstraccao ETA/MASTER
  components/state_machine/  README.txt -- FSM T/Tc/cascata
  components/radar_manager/  README.txt -- HLK-LD2450
  components/dali_manager/   README.txt -- PWM LEDC
  components/display_manager/README.txt -- LVGL display

================================================================================
11. SIMULADOR
================================================================================
  simulador/simulador_v6.html
  Abre directamente no browser. Sem dependencias externas.
  Implementa a logica completa: T/Tc, cascata, MASTER_CLAIM,
  falha de poste, restauro, fluxo de trafego intenso.

================================================================================