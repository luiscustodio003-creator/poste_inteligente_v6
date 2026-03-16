/* ============================================================
   SYSTEM CONFIG — PARAMETROS GLOBAIS DO SISTEMA
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Ficheiro central de configuracao do sistema.
   E o UNICO ficheiro que deve ser editado ao flashar
   cada poste. Todos os outros modulos importam daqui.

   Como configurar um novo poste:
   --------------------------------
   Editar APENAS as 3 linhas marcadas com [EDITAR]:
     - POSTE_ID       -> numero unico na rede (1, 2, 3, ...)
     - POSTE_NAME     -> nome para display e logs
     - POST_POSITION  -> posicao na cadeia (0=primeiro)
   O resto e IDENTICO em todos os postes.
============================================================ */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

/* ===========================================================
   IDENTIFICACAO DO POSTE
   -----------------------------------------------------------
   [EDITAR] estas 3 linhas em cada poste antes de flashar.
   =========================================================== */
#define POSTE_ID            1               /* [EDITAR] ID unico (1-255)        */
#define POSTE_NAME          "POSTE 01"      /* [EDITAR] Nome para display e logs */
#define POST_POSITION       0               /* [EDITAR] Posicao na cadeia        */

/* Aliases internos -- nao editar */
#define POST_ID             POSTE_ID
#define POST_NAME           POSTE_NAME

/* ===========================================================
   WI-FI STA — Rede comum (hotspot ou router)
   -----------------------------------------------------------
   Todos os postes ligam ao mesmo hotspot/router.
   IDENTICO em todos os postes.

   Broadcast por tipo de hotspot:
     Android : "192.168.43.255"
     iPhone  : "172.20.10.255"
     Router  : "192.168.1.255" ou "192.168.0.255"
   =========================================================== */
#define WIFI_SSID               "NomeDaRedeWifi"
#define WIFI_PASS               "PasswordDaRede"
#define WIFI_RETRY_ATTEMPTS     5           /* Tentativas STA antes de criar AP  */
#define WIFI_RECONNECT_MS       30000       /* Tenta reconectar em background    */

/* ===========================================================
   ACCESS POINT — Fallback se hotspot nao disponivel
   -----------------------------------------------------------
   Cada poste tem IP diferente para evitar conflito no fallback.
   POSTE 01: NETWORK_IP = "192.168.4.1"
   POSTE 02: NETWORK_IP = "192.168.4.2"
   ...
   =========================================================== */
#define NETWORK_SSID            "Poste_01"
#define NETWORK_PASS            "poste1234"
#define NETWORK_IP              "192.168.4.1"
#define NETWORK_BROADCAST       "192.168.43.255"

/* ===========================================================
   UDP — PROTOCOLO DE COMUNICACAO ENTRE POSTES
   -----------------------------------------------------------
   Porta partilhada por todos os postes.
   Broadcast para DISCOVER e MASTER_CLAIM.
   Unicast para SPD, ACK, STATUS, TC_INC.

   Protocolo de mensagens (texto simples, separado por ':'):

   DISCOVER:<id>:<nome>:<pos>
     Broadcast de anuncio periodico. Receptor responde com ACK.

   DISCOVER_ACK:<id>:<nome>:<pos>:<ip>
     Resposta unicast ao DISCOVER.

   SPD:<id>:<nome>:<pos>:<vel>:<eta_ms>:<dist_m>
     Velocidade detectada + ETA para o proximo poste.
     Receptor incrementa Tc e agenda acendimento.

   TC_INC:<id>:<pos>:<vel>
     Incrementa Tc no vizinho informado via UDP.
     Enviado quando o radar deteta carro antes do SPD chegar.

   ACK:<id>:<msg_type>
     Confirmacao de recepcao.

   STATUS:<id>:<pos>:<estado>
     Propagacao de estado: OK / OFFLINE / SAFE / MASTER

   MASTER_CLAIM:<id>:<pos>
     Poste reclama lideranca da linha (pos=0 tem prioridade).
     Propagado em cadeia de poste em poste.
   =========================================================== */
#define UDP_PORT                5005
#define MAX_NEIGHBORS           4
#define MAX_IP_LEN              16
#define ACK_TIMEOUT_MS          500
#define NEIGHBOR_TIMEOUT_MS     10000
#define DISCOVER_INTERVAL_MS    5000

/* ===========================================================
   FISICA DA INSTALACAO
   -----------------------------------------------------------
   Distancia real entre postes consecutivos na estrada.
   Alcance do radar HLK-LD2450 (detecta a ~7 metros).
   Usados para calcular ETA:
     ETA_ms = (POSTE_DIST_M / velocidade_ms) * 1000
   =========================================================== */
#define POSTE_DIST_M            50          /* Distancia entre postes (metros)   */
#define RADAR_DETECT_M          7           /* Alcance do radar HLK-LD2450 (m)   */

/* ===========================================================
   RADAR HLK-LD2450
   -----------------------------------------------------------
   USE_RADAR = 0 -> modo simulado (desenvolvimento/teste)
   USE_RADAR = 1 -> hardware UART real (producao)

   Ref: https://github.com/hermannsblum/ld2450
   =========================================================== */
#define USE_RADAR               0

/* ===========================================================
   ILUMINACAO — Niveis de brilho (%)
   -----------------------------------------------------------
   LIGHT_MIN      : brilho minimo em repouso (T=0, Tc=0)
   LIGHT_MAX      : brilho maximo com veiculo presente
   LIGHT_SAFE_MODE: brilho em modo seguro (radar falhou)
   TRAFIC_TIMEOUT_MS: tempo apos carro sair antes de voltar a MIN
   =========================================================== */
#define LIGHT_MIN               10          /* Brilho minimo em repouso (%)      */
#define LIGHT_MAX               100         /* Brilho maximo com veiculo (%)     */
#define LIGHT_SAFE_MODE         50          /* Brilho em modo seguro (%)         */
#define TRAFIC_TIMEOUT_MS       5000        /* Espera apos ultimo carro (ms)     */
#define DETECTION_TIMEOUT_MS    1000        /* Timeout de confirmacao radar (ms) */

/* ===========================================================
   DISPLAY — Actualizacao
   =========================================================== */
#define DISPLAY_UPDATE_MS       500         /* Actualizacao display (ms)         */

#endif /* SYSTEM_CONFIG_H */