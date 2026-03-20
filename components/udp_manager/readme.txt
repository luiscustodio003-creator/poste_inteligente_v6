================================================================================
 MÓDULO: udp_manager
 VERSÃO: 3.0
 DATA  : 2026-03-20
 AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRIÇÃO
---------
Gere toda a comunicação UDP entre postes da cadeia inteligente.
Implementa descoberta automática de vizinhos e o protocolo completo
de mensagens para coordenação de tráfego e gestão de falhas.


PROTOCOLO DE MENSAGENS
-----------------------
Todas as mensagens são texto simples separado por ':'.
Porto: UDP_PORT (definido em system_config.h, por omissão 5005).

  DISCOVER:<id>
      Broadcast periódico. Todos os postes actualizam a tabela
      de vizinhos ao receber. Resposta automática com STATUS:OK.
      Intervalo: DISCOVER_INTERVAL_MS (system_config.h)

  SPD:<id>:<vel>:<eta_ms>:<dist_m>
      Enviado ao vizinho direito quando radar detecta carro.
      O ETA (ms) permite ao vizinho acender no momento exacto.

  TC_INC:<id>:<vel>
      Enviado ao vizinho direito imediatamente após detecção.
      vel > 0 → carro a caminho (Tc++)
      vel < 0 → carro passou    (T--)

  STATUS:<id>:<estado>
      Propaga estado: OK | FAIL | SAFE | AUTO | OFFLINE
      Enviado em resposta a DISCOVER e quando estado muda.

  MASTER_CLAIM:<id>
      Enviado pelo poste pos=0 ao voltar online.
      O vizinho direito cede liderança e propaga em cadeia.


FUNÇÕES PÚBLICAS
----------------

  udp_manager_init()
      Cria socket, activa broadcast, faz bind e arranca task UDP.
      Chamar UMA VEZ após Wi-Fi com IP obtido.
      Retorna false se o socket falhar.

  udp_manager_discover()
      Envia DISCOVER broadcast manualmente.
      A task interna já chama periodicamente — uso opcional.

  udp_manager_send_spd(ip, speed, eta_ms, dist_m)
      Envia SPD a um IP específico.

  udp_manager_send_tc_inc(ip, speed)
      Envia TC_INC. speed negativa = sinal de T--.

  udp_manager_send_status(ip, status)
      Envia STATUS com enum neighbor_status_t.

  udp_manager_send_master_claim(ip)
      Envia MASTER_CLAIM a um IP específico.

  udp_manager_get_neighbors(nebL, nebR)
      Preenche buffers com IP dos vizinhos esq/dir.
      "---" se não houver vizinho conhecido.
      Compatível com main.c v2.4.

  udp_manager_get_neighbor_by_pos(position)
      Devolve ponteiro para neighbor_t com aquela posição.
      NULL se não encontrado. Usado pelo comm_manager.

  udp_manager_get_all_neighbors(list, max)
      Copia todos os vizinhos activos para o array.
      Devolve número de entradas copiadas.


CALLBACKS (implementar em state_machine.c)
------------------------------------------
Estas funções são declaradas __attribute__((weak)) no udp_manager.c.
A state_machine.c deve definir as versões reais:

  void on_tc_inc_received(float speed)
      Chamado ao receber TC_INC com speed > 0 (Tc++)

  void on_prev_passed_received(void)
      Chamado ao receber TC_INC com speed < 0 (T--)

  void on_spd_received(float speed, uint32_t eta_ms)
      Chamado ao receber SPD

  void on_master_claim_received(int from_id)
      Chamado ao receber MASTER_CLAIM


ESTRUTURA neighbor_t
---------------------
  char              ip[16]      IP do vizinho "x.x.x.x"
  int               id          ID do poste vizinho
  int               position    Posição na cadeia
  neighbor_status_t status      OK / OFFLINE / FAIL / SAFE / AUTO
  bool              active      true se entrada válida
  uint32_t          last_seen   Timestamp última recepção (ms)


DEPENDÊNCIAS
------------
  system_config.h  UDP_PORT, MAX_NEIGHBORS, DISCOVER_INTERVAL_MS,
                   NEIGHBOR_TIMEOUT_MS, POSTE_ID, POST_POSITION
  lwip             sockets UDP
  esp_timer        timestamps
  freertos         task UDP


COMO TESTAR (2 ESP32 na mesma rede)
-------------------------------------
  1. Flashar POSTE_ID=1 num ESP32 e POSTE_ID=2 noutro
  2. Ligar os dois à mesma rede Wi-Fi
  3. Abrir monitor série em ambos
  4. Após ~5s deves ver nos logs:
       [UDP_MGR] DISCOVER ID=2 IP=192.168.x.x [DIR]   (no poste 1)
       [UDP_MGR] DISCOVER ID=1 IP=192.168.x.x [ESQ]   (no poste 2)
  5. No display de cada poste, a zona de vizinhos fica azul "OK"
  6. Para testar TC_INC: descomentar bloco de teste no main.c


ALTERAÇÕES v2.1 → v3.0
------------------------
  + neighbor_t expandido: .id, .position, .status, .active
  + udp_manager_init() retorna bool (era void)
  + udp_manager_discover() exposta publicamente
  + udp_manager_send_spd() — nova
  + udp_manager_send_tc_inc() — nova
  + udp_manager_send_status() — nova
  + udp_manager_send_master_claim() — nova
  + udp_manager_get_neighbor_by_pos() — nova
  + udp_manager_get_all_neighbors() — nova
  + Callbacks weak: on_tc_inc_received, on_prev_passed_received,
                    on_spd_received, on_master_claim_received
  + neighbor_status_t definida neste módulo
================================================================================