/* ============================================================
   DALI MANAGER — DECLARACAO v2.0
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Controlo da luminaria via PWM LEDC com fade gradual.
   Implementa curva logaritmica DALI IEC 62386 (arc power 0-254)
   e fade por hardware LEDC (ledc_set_fade_with_time).

   Fade de subida: inversamente proporcional a velocidade do veiculo.
   Fade de descida: 4000ms fixo (suave, nao perturbador).

   NOTA: LEDC_HIGH_SPEED_MODE apenas no ESP32 original (LX6).
   Para ESP32-S2/S3/C3 alterar para LEDC_LOW_SPEED_MODE.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html
   Ref: https://www.dali-alliance.org/dali/ (IEC 62386)

   Dependencias:
   -------------
   - hw_config.h     (LED_PWM_PIN)
   - system_config.h (LIGHT_MIN, LIGHT_MAX, LIGHT_SAFE_MODE)
============================================================ */

#ifndef DALI_MANAGER_H
#define DALI_MANAGER_H

#include <stdint.h>

/* ----------------------------------------------------------
   INICIALIZACAO
---------------------------------------------------------- */

/* Inicializa timer LEDC, canal PWM e servico de fade */
void    dali_init(void);

/* ----------------------------------------------------------
   CONTROLO INSTANTANEO (sem fade)
   Usado em: init, SAFE_MODE, AUTONOMO
---------------------------------------------------------- */

/* Define brilho 0-100% de forma instantanea -- clamped */
void    dali_set_brightness(uint8_t brightness);

/* Liga a LIGHT_MAX (instantaneo) */
void    dali_turn_on(void);

/* Desliga para LIGHT_MIN (instantaneo) */
void    dali_turn_off(void);

/* Modo seguro -- LIGHT_SAFE_MODE (instantaneo, critico) */
void    dali_safe_mode(void);

/* ----------------------------------------------------------
   FADE GRADUAL IEC 62386
   Curva logaritmica: passos iguais = brilho percebido igual
---------------------------------------------------------- */

/*
 * Sobe para LIGHT_MAX com rampa dependente da velocidade:
 *   vel >= 80 km/h  -> 300ms   (veiculo rapido)
 *   vel == 50 km/h  -> 500ms
 *   vel <= 30 km/h  -> 800ms   (veiculo lento)
 *   vel == 0        -> 500ms   (default)
 */
void    dali_fade_up(float vel_kmh);

/*
 * Desce para LIGHT_MIN em 4000ms.
 * Usado quando T=0 e Tc=0 apos TRAFIC_TIMEOUT.
 */
void    dali_fade_down(void);

/*
 * Para fade em curso e mantem brilho no nivel actual.
 * Util quando novo veiculo chega durante fade de descida.
 */
void    dali_fade_stop(void);

/* ----------------------------------------------------------
   LEITURA DE ESTADO
---------------------------------------------------------- */

/* Retorna brilho actual (thread-safe via spinlock) */
uint8_t dali_get_brightness(void);

#endif /* DALI_MANAGER_H */