/* ============================================================
   DALI MANAGER — IMPLEMENTACAO
   ============================================================
   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Descricao:
   ----------
   Controlo PWM da luminaria via modulo LEDC do ESP32.
   Preparado para protocolo DALI real -- actualmente usa
   PWM analogico de 8 bits a 5000 Hz.

   Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html
============================================================ */

#include "dali_manager.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "DALI_MGR";

/* Configuracao do canal LEDC */
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define LEDC_FREQ_HZ    5000

/* Estado interno protegido por spinlock */
static uint8_t      s_brightness = 0;
static portMUX_TYPE s_mux        = portMUX_INITIALIZER_UNLOCKED;

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

    /* Arranca com brilho minimo */
    dali_set_brightness(LIGHT_MIN);

    ESP_LOGI(TAG, "DALI inicializado | pino=%d | %d%%",
             LED_PWM_PIN, LIGHT_MIN);
}

/* ===========================================================
   CONTROLO DE BRILHO
   =========================================================== */

void dali_set_brightness(uint8_t brightness)
{
    /* Limita entre LIGHT_MIN e LIGHT_MAX */
    if (brightness < LIGHT_MIN) brightness = LIGHT_MIN;
    if (brightness > LIGHT_MAX) brightness = LIGHT_MAX;

    /* Escala 0-100% para duty 0-255 (8 bits) */
    uint32_t duty = (uint32_t)((brightness * 255U) / 100U);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    /* Guarda estado de forma thread-safe */
    portENTER_CRITICAL(&s_mux);
    s_brightness = brightness;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGD(TAG, "Brilho: %d%% (duty=%lu)", brightness, duty);
}

/* Liga a potencia maxima */
void dali_turn_on(void)
{
    dali_set_brightness(LIGHT_MAX);
}

/* Desliga para potencia minima */
void dali_turn_off(void)
{
    dali_set_brightness(LIGHT_MIN);
}

/* Modo seguro -- brilho intermedio fixo */
void dali_safe_mode(void)
{
    dali_set_brightness(LIGHT_SAFE_MODE);
}

/* Retorna brilho actual de forma thread-safe */
uint8_t dali_get_brightness(void)
{
    uint8_t val;
    portENTER_CRITICAL(&s_mux);
    val = s_brightness;
    portEXIT_CRITICAL(&s_mux);
    return val;
}