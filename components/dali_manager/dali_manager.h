/* ============================================================
   DALi MANAGER MODULE
   ------------------------------------------------------------
   @file dali_manager.h
   @brief Controlo da luminária via PWM (preparado para DALi)

   Projeto: Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)
   ============================================================ */

#ifndef DALI_MANAGER_H
#define DALI_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/** Inicializa timer LEDC e canal PWM no pino LED_PWM_PIN. */
void    dali_init(void);

/** Define brilho (0-100%). Valores fora do intervalo são truncados
 *  para LIGHT_MIN / LIGHT_MAX definidos em system_config.h. */
void    dali_set_brightness(uint8_t brightness);

/** Liga luminária a LIGHT_MAX. */
void    dali_turn_on(void);

/** Desliga luminária para LIGHT_MIN. */
void    dali_turn_off(void);

/** Ativa modo de segurança (LIGHT_SAFE_MODE). */
void    dali_safe_mode(void);

/** Retorna brilho atual em % (thread-safe). */
uint8_t dali_get_brightness(void);

#endif /* DALI_MANAGER_H */
