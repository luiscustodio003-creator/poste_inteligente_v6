================================================================================
MODULO: dali_manager
VERSAO: 2.0 | DATA: 2026-03-31
PROJECTO: Poste Inteligente v6
AUTORES: Luis Custodio | Tiago Moreno
================================================================================

DESCRICAO
---------
Controla a luminaria do poste via PWM (LEDC do ESP32) com fade gradual.
Implementa a curva logaritmica do protocolo DALI IEC 62386 e usa o servico
de fade por hardware do ESP32 (ledc_set_fade_with_time), sem ocupar a CPU.

O QUE FAZ
---------
  - Inicializa canal LEDC (PWM) no GPIO 26 a 5000 Hz, 8 bits
  - Instala servico de fade LEDC por hardware
  - Converte percentagem para duty usando curva logaritmica DALI IEC 62386
  - Fade de subida: velocidade-dependente (30 km/h=800ms, 50=500ms, 80+=300ms)
  - Fade de descida: sempre 4000ms (suave, nao perturbador para condutores)
  - Controlo instantaneo para SAFE_MODE e init (sem fade)
  - Thread-safe: portENTER_CRITICAL / portEXIT_CRITICAL

NORMA DALI IEC 62386 — CURVA LOGARITMICA
------------------------------------------
  O protocolo DALI define arc power levels 0-254 com espassamento logaritmico.
  Cada passo produz a mesma variacao de brilho percebida (lei de Weber-Fechner).

  Formula de conversao (pct 0-100 -> arc 0-254 -> duty 0-255):
    arc = 1 + (253/3) * log10(pct * 10)   [para pct > 0]
    duty = round(arc * 255 / 254)

  Mapeamento resultante:
    pct=  1% -> arc~ 85  -> duty~ 85  (percebido: muito escuro)
    pct= 10% -> arc~170  -> duty~170  (percebido: escuro)
    pct= 50% -> arc~229  -> duty~230  (percebido: medio)
    pct=100% -> arc=254  -> duty=255  (percebido: maximo)

  Efeito pratico: subir de 10% para 50% "parece igual" a subir de 50% para 100%.
  Sem esta curva, PWM linear apareceria muito brilhante nos valores baixos
  e com pouca diferenca percebida nos valores altos.

FADE DE SUBIDA — dali_fade_up(vel_kmh)
----------------------------------------
  Tempo inversamente proporcional a velocidade do veiculo:
    vel >= 80 km/h  -> 300ms  (veiculo rapido: luz quase imediata)
    vel == 50 km/h  -> 500ms  (velocidade tipica)
    vel <= 30 km/h  -> 800ms  (veiculo lento: fade gradual)
    vel == 0        -> 500ms  (default — vel desconhecida)
  Formula: t_ms = 800 - (vel - 30) * 10, clamped [300, 800] ms

  Logica: um carro a 80 km/h percorre 2.2m em 100ms — luz instantanea
  e critica. A 30 km/h ha margem para fade confortavel.

FADE DE DESCIDA — dali_fade_down()
------------------------------------
  Sempre 4000ms para LIGHT_MIN (10%).
  Descida lenta evita contraste brusco para condutores ainda na zona.
  Chamado apos TRAFIC_TIMEOUT_MS sem deteccao (T=0 e Tc=0).

API PUBLICA
-----------
  void    dali_init(void)                    -- inicia LEDC + fade service
  void    dali_set_brightness(uint8_t pct)   -- instantaneo (SAFE, init)
  void    dali_turn_on(void)                 -- LIGHT_MAX instantaneo
  void    dali_turn_off(void)                -- LIGHT_MIN instantaneo
  void    dali_safe_mode(void)               -- LIGHT_SAFE_MODE instantaneo
  void    dali_fade_up(float vel_kmh)        -- rampa subida vel-dependente
  void    dali_fade_down(void)               -- rampa descida 4000ms
  void    dali_fade_stop(void)               -- para fade no nivel actual
  uint8_t dali_get_brightness(void)          -- brilho actual (%)

PINOS
-----
  GPIO 26 -- sinal PWM para dimmer ou rele de controlo da luminaria
  Frequencia: 5000 Hz
  Resolucao : 8 bits (duty 0-255)
  LEDC canal: LEDC_CHANNEL_0
  LEDC timer: LEDC_TIMER_0, LEDC_HIGH_SPEED_MODE (ESP32 LX6)

NIVEIS DE BRILHO (system_config.h)
------------------------------------
  LIGHT_MIN        10%   -- minimo (nunca desliga -- seguranca viaria)
  LIGHT_MAX       100%   -- maximo (veiculo presente)
  LIGHT_SAFE_MODE  50%   -- modo seguro (falha do radar)

INTEGRACAO COM STATE MACHINE
------------------------------
  Chamadas na state_machine.c:
    dali_fade_up(vel)     -- quando radar detecta carro (sm_on_radar_detect)
    dali_fade_up(vel)     -- quando SPD chega com ETA curto (on_spd_received)
    dali_fade_up(vel)     -- quando timer pre-ETA dispara
    dali_fade_up(vel)     -- fallback TC_INC sem SPD >1s
    dali_fade_down()      -- quando T=0,Tc=0 apos TRAFIC_TIMEOUT_MS
    dali_set_brightness() -- SAFE_MODE (instantaneo, critico)
    dali_set_brightness() -- init (sem fade no boot)

DEPENDENCIAS
------------
  hw_config.h     -- LED_PWM_PIN (GPIO 26)
  system_config.h -- LIGHT_MIN, LIGHT_MAX, LIGHT_SAFE_MODE
  driver/ledc.h   -- API LEDC do ESP-IDF (fade por hardware)
  math.h          -- log10f() para curva IEC 62386
  freertos        -- spinlock para thread-safety

NOTAS DE IMPLEMENTACAO
-----------------------
  ledc_fade_func_install(0) deve ser chamado UMA VEZ em dali_init().
  ledc_set_fade_with_time() cancela automaticamente qualquer fade anterior.
  ledc_fade_stop() e chamado em dali_set_brightness() antes do duty directo.

  O brilho minimo e 10% por seguranca rodoviaria (nunca apaga totalmente).
  LEDC_HIGH_SPEED_MODE e especifico do ESP32 original (LX6).
  Para ESP32-S2/S3/C3 usar LEDC_LOW_SPEED_MODE.

REFERENCIAS
-----------
  ESP-IDF LEDC (PWM + Fade):
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html

  Protocolo DALI IEC 62386 (DALI Alliance):
    https://www.dali-alliance.org/dali/

  IEC 62386 — Standard for Digital Addressable Lighting Interface:
    https://webstore.iec.ch/publication/6665

  GitHub — ESP-IDF exemplos LEDC (fade):
    https://github.com/espressif/esp-idf/tree/master/examples/peripherals/ledc

  YouTube — ESP32 LEDC PWM Fade (Espressif):
    https://www.youtube.com/@EspressifSystems

  YouTube — DALI Lighting Protocol Explained:
    Pesquisar "DALI protocol IEC 62386 explained" no YouTube
================================================================================
