/* ============================================================
   DISPLAY MANAGER — DECLARAÇÃO v5.1
   ------------------------------------------------------------
   @file      display_manager.h
   @brief     Interface pública do gestor de ecrã LVGL
   @version   5.1
   @date      2026-03-25

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF v5.x)

   Descrição:
   ----------
   Versão 5.1 — correcção de conflito de tipos com o
   radar_manager. Alterações em relação à v5.0:

   CORRECÇÃO PRINCIPAL (v5.0 → v5.1):
   ------------------------------------
   1. radar_obj_t REMOVIDA deste ficheiro.
      Era definida aqui E em radar_manager.h, causando:
        "conflicting types for 'radar_obj_t'"
      Solução: incluir radar_manager.h e usar o tipo de lá.

   2. radar_manager_get_objects() REMOVIDA deste ficheiro.
      Era declarada aqui E em radar_manager.h com tipos
      incompatíveis (RADAR_TRAIL_MAX vs RADAR_TRAIL_MAX_OBJ).
      Solução: a declaração existe apenas em radar_manager.h.

   3. RADAR_TRAIL_MAX e RADAR_MAX_OBJ definidos apenas em
      radar_manager.h. Este ficheiro não os redefine.

   Regra arquitectural após correcção:
   ------------------------------------
     radar_manager.h → define radar_obj_t (FONTE ÚNICA)
     display_manager.h → #include "radar_manager.h" (consumidor)
     main.c → inclui ambos sem conflito

   Pré-requisito de inicialização (ordem obrigatória):
   ----------------------------------------------------
     1. nvs_flash_init()
     2. post_config_init()
     3. display_manager_init()

   Configuração lv_conf.h necessária:
   ------------------------------------
     #define LV_USE_CANVAS          1   (OBRIGATÓRIO v5.0+)
     #define LV_USE_BAR             1
     #define LV_USE_LABEL           1
     #define LV_FONT_MONTSERRAT_10  1
     #define LV_FONT_MONTSERRAT_12  1
     #define LV_FONT_MONTSERRAT_14  1
     #define LV_FONT_MONTSERRAT_18  1

   Dependências directas:
   ----------------------
     radar_manager.h : radar_obj_t, RADAR_MAX_OBJ, RADAR_TRAIL_MAX
     st7789.h        : driver SPI do display
     system_config.h : LCD_H_RES, LCD_V_RES, LIGHT_MIN
     hw_config.h     : LCD_PIN_*
     post_config.h   : post_get_name(), post_get_id()
     lvgl            : v8.3.x

   Módulos que chamam este:
   ------------------------
     state_machine.c  → set_status(), set_leader()
     wifi_manager.c   → set_wifi()
     comm_manager.c   → set_neighbors()
     radar_manager.c  → (não chama — é o produtor dos dados)
     main.c           → init(), tick(), task(), set_radar()

============================================================ */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * INCLUSÃO DA FONTE ÚNICA DE VERDADE PARA radar_obj_t.
 *
 * Este include traz para display_manager.h:
 *   - radar_obj_t       (estrutura de objecto do radar)
 *   - RADAR_MAX_OBJ     (máximo de objectos simultâneos = 3)
 *   - RADAR_TRAIL_MAX   (comprimento do rasto = 8)
 *   - radar_manager_get_objects() (declaração da função)
 *
 * NÃO redefinir radar_obj_t nem radar_manager_get_objects()
 * neste ficheiro — causaria o conflito de tipos corrigido.
 */
#include "radar_manager.h"

/* ============================================================
   FUNÇÕES DE CICLO DE VIDA
============================================================ */

/**
 * @brief Inicializa ST7789, LVGL e interface v5.0.
 *        Chamar UMA VEZ em app_main(), depois de
 *        nvs_flash_init() e post_config_init().
 */
void display_manager_init(void);

/**
 * @brief Incrementa ticker LVGL. Chamar a cada 1 ms.
 * @param ms Milissegundos decorridos desde a última chamada.
 */
void display_manager_tick(uint32_t ms);

/**
 * @brief Processa eventos LVGL. Chamar a cada 5-10 ms.
 */
void display_manager_task(void);

/* ============================================================
   FUNÇÕES DE ACTUALIZAÇÃO DE ESTADO
============================================================ */

/**
 * @brief Actualiza badge de modo FSM (zona identidade).
 *        Estados: "IDLE", "LIGHT ON", "SAFE MODE",
 *                 "MASTER", "AUTONOMO".
 * @param status String do estado (não pode ser NULL).
 */
void display_manager_set_status(const char *status);

/**
 * @brief Atalho: true → badge "MASTER" verde; false → "IDLE".
 * @param is_leader Estado de liderança da cadeia.
 */
void display_manager_set_leader(bool is_leader);

/**
 * @brief Actualiza estado Wi-Fi e IP (linha 1 zona hardware).
 * @param connected true se Wi-Fi ligado com IP obtido.
 * @param ip        String do IP (pode ser NULL se desligado).
 */
void display_manager_set_wifi(bool connected, const char *ip);

/**
 * @brief Actualiza estado radar e brilho DALI.
 * @param radar_st   String do estado: "REAL", "SIM", ou "FAIL".
 * @param radar_ok   true se radar responde (REAL ou SIM); false = FAIL.
 * @param brightness Brilho DALI actual em percentagem (0-100).
 */
void display_manager_set_hardware(const char *radar_st, bool radar_ok, uint8_t brightness);

/**
 * @brief Actualiza contadores de tráfego T e Tc.
 *        T=0 e Tc=0 → cards cinzentos (IDLE).
 *        T>0 ou Tc>0 → cards coloridos (LIGHT ON).
 * @param T  Veículos detectados localmente pelo radar (0-999).
 * @param Tc Veículos anunciados via UDP ainda a chegar (0-999).
 */
void display_manager_set_traffic(int T, int Tc);

/**
 * @brief Actualiza card de velocidade (km/h).
 *        speed=0 → card cinzento; speed>0 → card violeta.
 * @param speed Velocidade em km/h (0-300).
 */
void display_manager_set_speed(int speed);

/**
 * @brief Actualiza estado dos vizinhos UDP (linha 2 zona hardware).
 *        Online → texto verde; offline → texto vermelho.
 * @param nebL    IP do vizinho esquerdo (NULL ou "" → "---").
 * @param nebR    IP do vizinho direito  (NULL ou "" → "---").
 * @param leftOk  true se vizinho esquerdo está a responder.
 * @param rightOk true se vizinho direito está a responder.
 */
void display_manager_set_neighbors(const char *nebL,
                                   const char *nebR,
                                   bool        leftOk,
                                   bool        rightOk);

/**
 * @brief Actualiza o canvas do radar com os objectos detectados.
 *        Chamar a cada frame do HLK-LD2450 (USE_RADAR=1) ou
 *        a cada ciclo do simulador (USE_RADAR=0).
 *
 *        O tipo radar_obj_t vem de radar_manager.h (v5.1).
 *        Se count=0, o canvas fica vazio (sem pontos).
 *
 * @param objs  Array de radar_obj_t preenchido pelo radar_manager
 *              ou pelo simulador. Pode ser NULL se count=0.
 * @param count Número de objectos válidos em objs (0..RADAR_MAX_OBJ).
 */
void display_manager_set_radar(const radar_obj_t *objs,
                               uint8_t            count);

#endif /* DISPLAY_MANAGER_H */