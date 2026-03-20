/* ============================================================
   DISPLAY MANAGER — INTERFACE
   ------------------------------------------------------------
   @file      display_manager.h
   @brief     Camada de apresentação LVGL + ST7789 — ecrã 240x240
   @version   3.0
   @date      2026-03-20

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Gere o display físico ST7789 e a interface gráfica LVGL.
   Divide o ecrã em quatro zonas fixas:

     ┌─────────────────────────────┐
     │  ZONA IDENTIDADE            │  Nome do poste + badge de modo
     ├─────────────────────────────┤
     │  ZONA HARDWARE              │  WiFi · Radar · DALI %
     ├─────────────────────────────┤
     │  ZONA TRÁFEGO               │  T | Tc | Vel km/h
     ├─────────────────────────────┤
     │  ZONA VIZINHOS              │  Esq IP·estado / Dir IP·estado
     └─────────────────────────────┘

   Notas de utilização:
   ---------------------
   - LVGL não é thread-safe: chamar display_manager_task()
     sempre a partir da mesma task (main_task ou display_task).
   - display_manager_tick() deve ser chamado a cada 1 ms
     (via esp_timer ou task dedicada).
   - Todas as funções set_* são seguras para chamar de qualquer
     task — usam apenas variáveis de estado; o LVGL actualiza
     no próximo ciclo de display_manager_task().

   Dependências:
   -------------
   - st7789.h         : driver SPI do display físico
   - system_config.h  : LCD_H_RES, LCD_V_RES, POSTE_NAME
   - hw_config.h      : LCD_PIN_*
   - lvgl             : biblioteca de UI (v8.3.x)

============================================================ */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================
   INICIALIZAÇÃO
============================================================ */

/**
 * @brief Inicializa o display ST7789, o LVGL e cria a UI completa.
 *        Deve ser chamada uma única vez em app_main(), antes de
 *        qualquer outra função deste módulo.
 */
void display_manager_init(void);

/* ============================================================
   CICLO DE VIDA LVGL
============================================================ */

/**
 * @brief Notifica o LVGL do tempo decorrido.
 *        Chamar periodicamente (ex: a cada 1 ms via timer).
 * @param ms Milissegundos decorridos desde a última chamada
 */
void display_manager_tick(uint32_t ms);

/**
 * @brief Processa eventos e timers internos do LVGL.
 *        Chamar periodicamente a partir da task de display
 *        (tipicamente a cada 5–10 ms).
 */
void display_manager_task(void);

/* ============================================================
   ZONA IDENTIDADE — nome do poste e modo da FSM
============================================================ */

/**
 * @brief Actualiza o badge de modo/estado da FSM.
 *        Texto e cor mudam conforme o estado:
 *          "IDLE"     → cinzento
 *          "LIGHT ON" → amarelo
 *          "SAFE MODE"→ laranja
 *          "MASTER"   → verde
 *          "AUTÓNOMO" → vermelho
 * @param status String do estado (ex: "IDLE", "LIGHT ON", ...)
 */
void display_manager_set_status(const char *status);

/**
 * @brief Activa ou desactiva indicador visual de liderança MASTER.
 *        Quando true, o badge fica verde com texto "MASTER".
 * @param is_leader true se este poste é o líder da cadeia
 */
void display_manager_set_leader(bool is_leader);

/* ============================================================
   ZONA HARDWARE — WiFi, Radar, DALI
============================================================ */

/**
 * @brief Actualiza o estado Wi-Fi e IP no display.
 * @param connected true se Wi-Fi ligado, false caso contrário
 * @param ip        String com o endereço IP (NULL → "---")
 */
void display_manager_set_wifi(bool connected, const char *ip);

/**
 * @brief Actualiza o estado do radar e o brilho DALI.
 * @param radar_ok   true se radar operacional, false se em falha
 * @param brightness Brilho actual da luminária (0–100 %)
 */
void display_manager_set_hardware(bool radar_ok, uint8_t brightness);

/* ============================================================
   ZONA TRÁFEGO — contadores T, Tc e velocidade
============================================================ */

/**
 * @brief Actualiza os contadores de tráfego T e Tc.
 * @param T  Carros confirmados pelo radar local (presentes)
 * @param Tc Carros a caminho anunciados via UDP
 */
void display_manager_set_traffic(int T, int Tc);

/**
 * @brief Actualiza a velocidade do último carro detectado.
 * @param speed Velocidade em km/h (0 = sem detecção activa)
 */
void display_manager_set_speed(int speed);

/* ============================================================
   ZONA VIZINHOS — estado da cadeia UDP
============================================================ */

/**
 * @brief Actualiza os IPs e estados dos vizinhos esq e dir.
 * @param nebL   IP do vizinho esquerdo (NULL → "---")
 * @param nebR   IP do vizinho direito  (NULL → "---")
 * @param leftOk  true se vizinho esquerdo está online
 * @param rightOk true se vizinho direito está online
 */
void display_manager_set_neighbors(const char *nebL,
                                   const char *nebR,
                                   bool        leftOk,
                                   bool        rightOk);

#endif /* DISPLAY_MANAGER_H */