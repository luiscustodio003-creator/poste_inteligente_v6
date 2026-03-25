/* ============================================================
   SYSTEM CONFIG — PARÂMETROS GLOBAIS
   ------------------------------------------------------------
   @file      system_config.h
   @brief     Único ficheiro a editar ao flashar cada poste.
              Define identidade, rede, radar e iluminação.
   

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)


   COMO CONFIGURAR UM NOVO POSTE:
   --------------------------------
   Editar apenas as 3 linhas marcadas com [EDITAR]:
       #define POSTE_ID       2
       #define POSTE_NAME     "POSTE 02"
       #define POST_POSITION  1
   Compilar e flashar — a NVS é populada automaticamente
   no primeiro arranque.
============================================================ */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

/* ============================================================
   IDENTIFICAÇÃO DO POSTE
   [EDITAR] estas 3 linhas são únicas por poste
============================================================ */
#define POSTE_ID            1       /* ID numérico único (uint8_t) */
#define POSTE_NAME          "POSTE 01"  /* Nome legível (max 31 chars) */
#define POST_POSITION       0      /* Posição na cadeia (0=início) */

/* ============================================================
   WI-FI STA (DHCP)
============================================================ */
#define WIFI_SSID               "wifi"   /* SSID da rede Wi-Fi      */
#define WIFI_PASS               "123456789"   /* Password da rede Wi-Fi  */
#define WIFI_RETRY_ATTEMPTS     5                /* Tentativas antes de desistir */
#define WIFI_RECONNECT_MS       30000            /* Intervalo reconexão background (ms) */

/* ============================================================
   DISPLAY
============================================================ */
#define LCD_H_RES               240     /* Resolução horizontal (pixels) */
#define LCD_V_RES               240     /* Resolução vertical (pixels)   */
#define DISPLAY_UPDATE_MS       500     /* Intervalo actualização display (ms) */

/* ============================================================
   PROTOCOLO UDP — DESCOBERTA DE VIZINHOS
   Fonte única de verdade: udp_manager.c usa estes valores
============================================================ */
#define UDP_PORT                5005    /* Porto UDP de escuta e envio   */
#define MAX_NEIGHBORS           4       /* Máximo de vizinhos conhecidos */
#define MAX_IP_LEN              16      /* Tamanho máximo string IP      */
#define ACK_TIMEOUT_MS          500     /* Timeout espera ACK (ms)       */
#define NEIGHBOR_TIMEOUT_MS     8000   /* Tempo sem sinal → vizinho expirado (ms) */
#define DISCOVER_INTERVAL_MS    2000    /* Intervalo entre broadcasts DISCOVER (ms) */

/* ============================================================
   FÍSICA / RADAR / ILUMINAÇÃO
============================================================ */
#define POSTE_DIST_M            50      /* Distância entre postes (metros)       */

/* Alcance máximo do HLK-LD2450 em metros.
   O sensor consegue detectar objectos até ~10m dependendo
   do tamanho do alvo e da reflexividade.
   Usado pelo simulador e pelo display (escala do canvas).
   RADAR_DETECT_M define a zona de acção para acender a luz —
   RADAR_MAX_M define o campo de visão total do sensor.        */
#define RADAR_MAX_M             10      /* Alcance máximo do radar (metros)      */
#define RADAR_MAX_MM            (RADAR_MAX_M * 1000)  /* Idem em milímetros     */

/* Distância a partir da qual o poste acende a 100%.
   Deve ser <= RADAR_MAX_M.
   Exemplo: carro detectado a 7m → luz acende imediatamente.  */
#define RADAR_DETECT_M          7       /* Zona de acção para acender (metros)   */

#define USE_RADAR               0       /* 0 = simulado | 1 = hardware HLK-LD2450 */
#define LIGHT_MIN               10      /* Brilho mínimo (%)                     */
#define LIGHT_MAX               100     /* Brilho máximo (%)                     */
#define LIGHT_SAFE_MODE         50      /* Brilho em modo seguro (%)             */
#define TRAFIC_TIMEOUT_MS       5000    /* Tempo após último carro → baixar brilho (ms) */
#define DETECTION_TIMEOUT_MS    1000    /* Timeout de detecção activa (ms)       */

#endif /* SYSTEM_CONFIG_H */