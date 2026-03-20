/* ============================================================
   SYSTEM CONFIG — PARÂMETROS GLOBAIS
   ------------------------------------------------------------
   @file      system_config.h
   @brief     Único ficheiro a editar ao flashar cada poste.
              Define identidade, rede, radar e iluminação.
   @version   2.1
   @date      2026-03-19

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
#define POSTE_ID            2         /* ID numérico único (uint8_t) */
#define POSTE_NAME          "POSTE 02"  /* Nome legível (max 31 chars) */
#define POST_POSITION       1          /* Posição na cadeia (0=início) */

/* ============================================================
   WI-FI STA (DHCP)
============================================================ */
#define WIFI_SSID               "Netdovizinho"   /* SSID da rede Wi-Fi      */
#define WIFI_PASS               "0325917ea6e4"   /* Password da rede Wi-Fi  */
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
#define NEIGHBOR_TIMEOUT_MS     10000   /* Tempo sem sinal → vizinho expirado (ms) */
#define DISCOVER_INTERVAL_MS    5000    /* Intervalo entre broadcasts DISCOVER (ms) */

/* ============================================================
   FÍSICA / RADAR / ILUMINAÇÃO
============================================================ */
#define POSTE_DIST_M            50      /* Distância entre postes (metros)       */
#define RADAR_DETECT_M          7       /* Alcance de detecção do radar (metros) */
#define USE_RADAR               0       /* 0 = simulado | 1 = hardware HLK-LD2450 */
#define LIGHT_MIN               10      /* Brilho mínimo (%)                     */
#define LIGHT_MAX               100     /* Brilho máximo (%)                     */
#define LIGHT_SAFE_MODE         50      /* Brilho em modo seguro (%)             */
#define TRAFIC_TIMEOUT_MS       5000    /* Tempo após último carro → baixar brilho (ms) */
#define DETECTION_TIMEOUT_MS    1000    /* Timeout de detecção activa (ms)       */

#endif /* SYSTEM_CONFIG_H */