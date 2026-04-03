================================================================================
MODULO: udp_manager
VERSAO: 3.3 | DATA: 2026-03-25
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Gere toda a comunicacao UDP entre postes da cadeia inteligente.
Implementa descoberta automatica de vizinhos e o protocolo completo
de mensagens para coordenacao de trafego e gestao de falhas.
Corre uma task dedicada (udp_task) para recepcao assincrona de mensagens.

PROTOCOLO DE MENSAGENS
-----------------------
  Todas as mensagens sao texto simples separado por ':'.
  Porto: UDP_PORT (definido em system_config.h, por omissao 5005).
  Envio: unicast para IP especifico (exceto DISCOVER = broadcast).

  DISCOVER:<id>
      Broadcast periodico (DISCOVER_INTERVAL_MS).
      Todos os postes actualizam tabela de vizinhos ao receber.
      Resposta automatica: STATUS:OK ao remetente.

  SPD:<id>:<vel>:<eta_ms>:<dist_m>
      Enviado ao vizinho direito apos deteccao de veiculo.
      vel     : velocidade em km/h
      eta_ms  : tempo estimado de chegada ao proximo poste (ms)
      dist_m  : distancia entre postes (metros)

  TC_INC:<id>:<vel>
      Enviado ao vizinho direito imediatamente apos deteccao.
      Activa on_tc_inc_received() no receptor -> Tc++

  STATUS:<id>:<estado>
      Propaga estado: OK | FAIL | SAFE | AUTO | OFFLINE
      Enviado em resposta a DISCOVER e quando estado muda.

  MASTER_CLAIM:<id>
      Enviado pelo poste pos=0 ao voltar online.
      Receptor cede lideranca e propaga em cadeia para a direita.

CALLBACKS (implementar em state_machine.c)
------------------------------------------
  Declarados __attribute__((weak)) -- a state_machine.c define as versoes reais.

  void on_tc_inc_received(float speed)
      Chamado ao receber TC_INC. speed > 0 = carro a caminho (Tc++)

  void on_prev_passed_received(void)
      Chamado ao receber notificacao de carro passou (T--)

  void on_spd_received(float speed, uint32_t eta_ms)
      Chamado ao receber SPD -- agenda pre-acendimento

  void on_master_claim_received(int from_id)
      Chamado ao receber MASTER_CLAIM -- cede lideranca

ESTRUTURA neighbor_t
---------------------
  char              ip[MAX_IP_LEN]   IP do vizinho "x.x.x.x\0"
  int               id               ID do poste vizinho
  int               position         Posicao na cadeia
  neighbor_status_t status           OK/OFFLINE/FAIL/SAFE/AUTO
  bool              active           true se entrada valida
  bool              discover_ok      true se ja respondeu a DISCOVER
  uint32_t          last_seen        Timestamp ultima recepcao (ms)

ENUM neighbor_status_t
------------------------
  NEIGHBOR_OK       -- poste a responder normalmente
  NEIGHBOR_OFFLINE  -- timeout (NEIGHBOR_TIMEOUT_MS sem mensagem)
  NEIGHBOR_FAIL     -- reportou falha de hardware
  NEIGHBOR_SAFE     -- em SAFE_MODE (radar falhou)
  NEIGHBOR_AUTO     -- em modo AUTONOMO (UDP falhou nesse poste)

API PUBLICA
-----------
  bool      udp_manager_init(void)
  void      udp_manager_discover(void)
  bool      udp_manager_send_spd(ip, speed, eta_ms, dist_m)
  bool      udp_manager_send_tc_inc(ip, speed)
  bool      udp_manager_send_status(ip, status)
  bool      udp_manager_send_master_claim(ip)
  void      udp_manager_get_neighbors(nebL, nebR)
  neighbor_t *udp_manager_get_neighbor_by_pos(int position)
  size_t    udp_manager_get_all_neighbors(list, max)

TIMINGS CRITICOS (system_config.h)
------------------------------------
  NEIGHBOR_TIMEOUT_MS    8000  -- sem mensagem -> vizinho OFFLINE
  DISCOVER_INTERVAL_MS   2000  -- intervalo entre broadcasts DISCOVER
  ACK_TIMEOUT_MS          500  -- timeout espera ACK (nao usado actualmente)

COMO TESTAR (2 ESP32 na mesma rede)
-------------------------------------
  1. Flashar POSTE_ID=1, POST_POSITION=0 num ESP32
  2. Flashar POSTE_ID=2, POST_POSITION=1 noutro ESP32
  3. Ligar os dois a mesma rede Wi-Fi
  4. Apos ~5s nos logs de cada poste:
       [UDP_MGR] DISCOVER ID=2 IP=x.x.x.x [DIR]    (no poste 1)
       [UDP_MGR] DISCOVER ID=1 IP=x.x.x.x [ESQ]    (no poste 2)
  5. No display: zona vizinhos fica verde

DEPENDENCIAS
------------
  system_config.h  -- UDP_PORT, MAX_NEIGHBORS, DISCOVER_INTERVAL_MS,
                      NEIGHBOR_TIMEOUT_MS, POSTE_ID, POST_POSITION, MAX_IP_LEN
  lwip/sockets.h   -- sockets UDP BSD (lwIP stack do ESP-IDF)
  esp_timer.h      -- timestamps
  freertos/task.h  -- task UDP dedicada

REFERENCIAS
-----------
  ESP-IDF Sockets (lwIP):
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/lwip.html

  ESP-IDF UDP Socket exemplo:
    https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_server

  lwIP documentacao:
    https://www.nongnu.org/lwip/2_1_x/group__socket.html

  YouTube - UDP Communication ESP32:
    Pesquisar "ESP32 UDP socket ESP-IDF" no YouTube

  YouTube - Espressif Networking:
    https://www.youtube.com/@EspressifSystems
================================================================================
