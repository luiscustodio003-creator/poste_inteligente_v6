/* ============================================================
   DALI MANAGER — IMPLEMENTACAO v2.0
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Controlo PWM da luminaria com fade gradual por hardware LEDC.

   Norma implementada: DALI IEC 62386
   ------------------------------------
   O protocolo DALI define 255 niveis de brilho (arc power 0-254)
   com curva logaritmica: cada passo produz a mesma variacao de
   brilho percebida (lei de Weber-Fechner).

   Formula de conversao (pct 0-100 -> arc 0-254):
     arc = 1 + (253/3) * log10(pct * 10)
   Mapeamento:
     pct=  1% -> arc~ 85  (percebido: muito escuro)
     pct= 10% -> arc~170  (percebido: escuro)
     pct= 50% -> arc~229  (percebido: medio)
     pct=100% -> arc=254  (percebido: maximo)
   Resultado: subir de 10% para 50% "parece" igual a 50% para 100%.

   Fade de subida (dali_fade_up):
   --------------------------------
   Tempo inversamente proporcional a velocidade do veiculo:
     30 km/h -> 800ms  | 50 km/h -> 500ms | 80+ km/h -> 300ms
   Logica: veiculo rapido precisa de luz quase imediata.

   Fade de descida (dali_fade_down):
   ------------------------------------
   Sempre 4000ms, independente da velocidade anterior.
   Descida lenta evita contraste brusco para condutores.

   Hardware: ledc_set_fade_with_time() — fade por interrupcao
   de hardware, sem ocupar a CPU.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html
   Ref: https://www.dali-alliance.org/dali/
============================================================ */

#include "dali_manager.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <math.h>

static const char *TAG = "DALI_MGR";

/* Configuracao do canal LEDC */
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define LEDC_FREQ_HZ    5000

/* Duracao do fade de descida (ms) */
#define FADE_DOWN_MS    4000U

/* Estado interno protegido por spinlock */
static uint8_t      s_brightness     = 0;
static bool         s_fade_installed = false;  /* true apos ledc_fade_func_install */
static portMUX_TYPE s_mux            = portMUX_INITIALIZER_UNLOCKED;

/* ===========================================================
   _pct_to_duty
   -----------------------------------------------------------
   Converte percentagem (0-100) para duty LEDC (0-255)
   usando a curva logaritmica DALI IEC 62386.

   Formula (arc power level):
     arc = 1 + (253/3) * log10(pct * 10)   [para pct > 0]
     arc = 0                                 [para pct == 0]

   Mapeamento (pct -> arc -> duty):
     pct=  1% -> arc~ 85  -> duty~ 85
     pct= 10% -> arc~170  -> duty~170
     pct= 50% -> arc~229  -> duty~230
     pct=100% -> arc= 254 -> duty=255

   Efeito: passos iguais de arc produzem passos iguais de
   brilho percebido (lei de Weber-Fechner / DALI IEC 62386).
=========================================================== */
static uint32_t _pct_to_duty(uint8_t pct)
{
    if (pct == 0)   return 0;
    if (pct >= 100) return 255;

    /* DALI arc level: 1..254 */
    float arc = 1.0f + (253.0f / 3.0f) * log10f((float)pct * 10.0f);
    if (arc < 0.0f)   arc = 0.0f;
    if (arc > 254.0f) arc = 254.0f;

    /* Mapeia arc 0-254 para duty 0-255 */
    return (uint32_t)((arc * 255.0f / 254.0f) + 0.5f);
}

/* ===========================================================
   _fade_to_pct  (interno)
   -----------------------------------------------------------
   Aplica fade de hardware LEDC para um dado brilho em
   percentagem, com uma duracao em milissegundos.
   NAO-BLOQUEANTE: usa LEDC_FADE_NO_WAIT.
   Chama automaticamente ledc_set_fade_with_time() que cancela
   qualquer fade anterior antes de iniciar o novo.
=========================================================== */
static void _fade_to_pct(uint8_t pct, uint32_t time_ms)
{
    if (pct < LIGHT_MIN) pct = LIGHT_MIN;
    if (pct > LIGHT_MAX) pct = LIGHT_MAX;

    uint32_t duty = _pct_to_duty(pct);

    if (s_fade_installed) {
        ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, duty, (int)time_ms);
        ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    } else {
        /* Fallback: fade service nao instalado -- aplica duty directo */
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }

    portENTER_CRITICAL(&s_mux);
    s_brightness = pct;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGD(TAG, "Fade -> %d%% em %lums (duty=%lu)",
             pct, (unsigned long)time_ms, (unsigned long)duty);
}

/* ===========================================================
   INICIALIZACAO
=========================================================== */
void dali_init(void)
{
    /* Configura timer LEDC */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    /* Configura canal no pino da luminaria */
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LED_PWM_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    /* Instala servico de fade por hardware
       (obrigatorio antes de qualquer ledc_set_fade_with_time) */
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    s_fade_installed = true;

    /* Arranca com brilho minimo (instantaneo -- sem fade no boot) */
    dali_set_brightness(LIGHT_MIN);

    ESP_LOGI(TAG, "DALI v2.0 | pino=%d | %d%% | fade LEDC | IEC 62386",
             LED_PWM_PIN, LIGHT_MIN);
}

/* ===========================================================
   CONTROLO INSTANTANEO
   Cancela fade em curso e aplica duty de imediato.
   Usado em: init, SAFE_MODE, AUTONOMO.
=========================================================== */
void dali_set_brightness(uint8_t brightness)
{
    if (brightness < LIGHT_MIN) brightness = LIGHT_MIN;
    if (brightness > LIGHT_MAX) brightness = LIGHT_MAX;

    uint32_t duty = _pct_to_duty(brightness);

    /* Cancela qualquer fade em curso iniciando um novo com 1ms.
       ledc_set_fade_with_time() substitui o fade activo imediatamente.
       Compativel com ESP-IDF v4.x e v5.x (ledc_fade_stop nao existe em v4). */
    if (s_fade_installed) {
        ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, duty, 1);
        ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    } else {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    }

    portENTER_CRITICAL(&s_mux);
    s_brightness = brightness;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGD(TAG, "Brilho: %d%% (duty=%lu)", brightness, (unsigned long)duty);
}

/* Liga a LIGHT_MAX instantaneamente */
void dali_turn_on(void)
{
    dali_set_brightness(LIGHT_MAX);
}

/* Desliga para LIGHT_MIN instantaneamente */
void dali_turn_off(void)
{
    dali_set_brightness(LIGHT_MIN);
}

/* Modo seguro -- brilho intermedio fixo (instantaneo por seguranca) */
void dali_safe_mode(void)
{
    dali_set_brightness(LIGHT_SAFE_MODE);
}

/* ===========================================================
   FADE GRADUAL IEC 62386 — API PUBLICA
=========================================================== */

/*
 * dali_fade_up
 * -----------------------------------------------------------
 * Sobe para LIGHT_MAX com rampa de tempo inversamente
 * proporcional a velocidade do veiculo detectado:
 *
 *   vel >= 80 km/h  -> 300ms  (veiculo rapido: luz quase imediata)
 *   vel == 50 km/h  -> 500ms
 *   vel <= 30 km/h  -> 800ms  (veiculo lento: fade confortavel)
 *   vel == 0        -> 500ms  (default quando vel desconhecida)
 *
 * Curva logaritmica IEC 62386 garante que a subida parece
 * uniforme ao observador (sem salto brusco no inicio).
 */
void dali_fade_up(float vel_kmh)
{
    uint32_t t_ms;

    if      (vel_kmh <= 0.0f)  t_ms = 500U;   /* default */
    else if (vel_kmh >= 80.0f) t_ms = 300U;
    else if (vel_kmh <= 30.0f) t_ms = 800U;
    else                       t_ms = (uint32_t)(800.0f - (vel_kmh - 30.0f) * 10.0f);

    ESP_LOGI(TAG, "Fade UP | %.0f km/h -> %lums -> %d%%",
             vel_kmh, (unsigned long)t_ms, LIGHT_MAX);

    _fade_to_pct(LIGHT_MAX, t_ms);
}

/*
 * dali_fade_down
 * -----------------------------------------------------------
 * Desce para LIGHT_MIN em FADE_DOWN_MS (4000ms).
 * Descida lenta evita contraste brusco para condutores
 * que possam ainda estar na zona de iluminacao do poste.
 */
void dali_fade_down(void)
{
    ESP_LOGI(TAG, "Fade DOWN | %ums -> %d%%", FADE_DOWN_MS, LIGHT_MIN);
    _fade_to_pct(LIGHT_MIN, FADE_DOWN_MS);
}

/*
 * dali_fade_stop
 * -----------------------------------------------------------
 * Para fade em curso e mantem brilho no nivel actual.
 * Util quando novo veiculo e detectado durante fade de descida.
 * Actualiza s_brightness com o duty real lido do hardware.
 */
void dali_fade_stop(void)
{
    if (!s_fade_installed) return;

    /* Le duty actual do hardware (valor instantaneo do fade em curso) */
    uint32_t duty_now = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL);

    /* Cancela o fade iniciando um novo fade para o mesmo valor em 1ms.
       ledc_set_fade_with_time() substitui o fade activo — funciona em v4.x e v5.x. */
    ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, duty_now, 1);
    ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

    uint8_t pct_now = (uint8_t)((duty_now * 100U) / 255U);
    if (pct_now < LIGHT_MIN) pct_now = LIGHT_MIN;

    portENTER_CRITICAL(&s_mux);
    s_brightness = pct_now;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGD(TAG, "Fade STOP | duty=%lu pct~%d%%",
             (unsigned long)duty_now, pct_now);
}

/* ===========================================================
   LEITURA DE ESTADO
=========================================================== */

/* Retorna brilho actual (target ou valor parado, thread-safe) */
uint8_t dali_get_brightness(void)
{
    uint8_t val;
    portENTER_CRITICAL(&s_mux);
    val = s_brightness;
    portEXIT_CRITICAL(&s_mux);
    return val;
}
