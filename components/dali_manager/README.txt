DALi Manager - Poste Inteligente
===============================

Descrição:
-----------
Substitui o LED Driver, controla a luminária via PWM.
Preparado para integração futura com protocolo DALi.

Funcionalidades:
- Inicialização do hardware PWM
- Ajuste de brilho (0-100%)
- Liga/Desliga
- SAFE MODE (50%)
- Integração com FSM e radar

Pinos:
-------
- LED_PWM_PIN (GPIO19) -> luminária principal
- PWM escolhido por ser seguro e compatível com ESP32 PWM
- SAFE MODE ajusta o brilho sem desligar completamente

Inclusão no projeto:
-------------------
#include "dali_manager.h"

Funções principais:
- dali_init()
- dali_set_brightness(uint8_t)
- dali_turn_on()
- dali_turn_off()
- dali_safe_mode()
- dali_get_brightness()