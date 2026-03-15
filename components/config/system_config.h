/* ============================================================
   SYSTEM CONFIG — PARAMETROS GLOBAIS
   ------------------------------------------------------------
   @file      system_config.h
   @brief     Parametros globais do sistema de Postes Inteligentes
   @version   2.0
   @date      2026-03-15

   Projecto  : Poste Inteligente — POSTE 02
   Plataforma: ESP32 (ESP-IDF)

   ATENCAO: Este ficheiro esta configurado para o POSTE 02.
   Diferencas em relacao ao Poste 01:
     - POSTE_ID = 2
     - POST_POSITION = 1
     - WIFI_SSID = "Poste_01"  (liga-se ao AP do Poste 01)
     - WIFI_PASS = "poste1234" (password do AP do Poste 01)
     - NETWORK_SSID = "Poste_02" (AP proprio se STA falhar)
     - NETWORK_IP = "192.168.4.2" (IP diferente do Poste 01)

   Ordem de arranque:
     1. Ligar Poste 01 primeiro -- aguardar AP activo
     2. Ligar Poste 02 -- liga-se ao AP do Poste 01 como STA
     3. Ambos ficam em 192.168.4.x -- comunicacao UDP activa
============================================================ */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

/* ===========================================================
   IDENTIFICACAO DO POSTE
   =========================================================== */
#define POSTE_ID            2
#define POSTE_NAME          "POSTE 02"
#define POST_ID             POSTE_ID
#define POST_NAME           POSTE_NAME
#define POST_POSITION       1

/* ===========================================================
   WI-FI STA — Liga-se ao AP do Poste 01
   -----------------------------------------------------------
   WIFI_SSID aponta para o AP criado pelo Poste 01.
   Se o Poste 01 nao estiver acessivel apos WIFI_RETRY_ATTEMPTS
   tentativas, o Poste 02 cria o seu proprio AP (NETWORK_SSID).
   =========================================================== */
#define WIFI_SSID               "Poste_01"
#define WIFI_PASS               "poste1234"
#define WIFI_RETRY_ATTEMPTS     5

/* ===========================================================
   ACCESS POINT — Rede propria (fallback se STA falhar)
   -----------------------------------------------------------
   IP diferente do Poste 01 (192.168.4.1) para evitar conflito.
   =========================================================== */
#define NETWORK_SSID            "Poste_01"
#define NETWORK_PASS            "poste1234"
#define NETWORK_IP              "192.168.4.1"
#define NETWORK_BROADCAST       "192.168.4.255"

/* ===========================================================
   UDP — Comunicacao entre postes
   =========================================================== */
#define UDP_PORT                5005
#define MAX_NEIGHBORS           4
#define MAX_IP_LEN              16
#define ACK_TIMEOUT_MS          500
#define NEIGHBOR_TIMEOUT_MS     10000
#define DISCOVER_INTERVAL_MS    5000

/* ===========================================================
   RADAR HLK-LD2450
   =========================================================== */
#define USE_RADAR               0

/* ===========================================================
   ILUMINACAO — Niveis de brilho (%)
   =========================================================== */
#define LIGHT_MIN               10
#define LIGHT_MAX               100
#define LIGHT_SAFE_MODE         50
#define LIGHT_ON_TIMEOUT_MS     5000
#define DETECTION_TIMEOUT_MS    1000

/* ===========================================================
   DISPLAY — Actualizacao
   =========================================================== */
#define DISPLAY_UPDATE_MS       2000

#endif /* SYSTEM_CONFIG_H */