/* ============================================================
   DALI MANAGER — DECLARACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Controlo da luminaria via PWM LEDC (simulacao DALI).
   Escala 0-100% para duty 0-255 (8 bits, 5000 Hz).
   Thread-safe via spinlock portMUX.

   NOTA: LEDC_HIGH_SPEED_MODE apenas no ESP32 original (LX6).
   Para ESP32-S2/S3/C3 alterar para LEDC_LOW_SPEED_MODE.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html

   Dependencias:
   -------------
   - hw_config.h (LED_PWM_PIN)
   - system_config.h (LIGHT_MIN, LIGHT_MAX, LIGHT_SAFE_MODE)
============================================================ */

#ifndef DALI_MANAGER_H
#define DALI_MANAGER_H

#include <stdint.h>

/* Inicializa timer LEDC e canal PWM */
void    dali_init(void);

/* Define brilho 0-100% -- clamped por LIGHT_MIN/MAX */
void    dali_set_brightness(uint8_t brightness);

/* Liga a LIGHT_MAX */
void    dali_turn_on(void);

/* Desliga para LIGHT_MIN */
void    dali_turn_off(void);

/* Modo seguro -- LIGHT_SAFE_MODE */
void    dali_safe_mode(void);

/* Retorna brilho actual (thread-safe via spinlock) */
uint8_t dali_get_brightness(void);

#endif /* DALI_MANAGER_H */