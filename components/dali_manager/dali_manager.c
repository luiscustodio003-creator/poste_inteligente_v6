/* ============================================================
   DALi MANAGER IMPLEMENTATION
   ------------------------------------------------------------
   @file dali_manager.c
   @brief Controlo de luminária via PWM (simulação DALi)

   Projeto: Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)
   ============================================================ */

#include "dali_manager.h"
#include "hw_config.h"
#include "system_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "DALI_MGR";

#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define LEDC_FREQ_HZ    5000

static uint8_t        g_current_brightness = 0;
static portMUX_TYPE   s_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ============================================================
   Inicialização
   ============================================================ */
void dali_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LED_PWM_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    ESP_LOGI(TAG, "DALi Manager inicializado no pino %d", LED_PWM_PIN);
}

/* ============================================================
   Controlo de brilho
   ============================================================ */
void dali_set_brightness(uint8_t brightness)
{
    if (brightness < LIGHT_MIN) brightness = LIGHT_MIN;
    if (brightness > LIGHT_MAX) brightness = LIGHT_MAX;

    /* Escala 0-100% para 0-255 (8 bits) */
    uint32_t duty = (uint32_t)((brightness * 255U) / 100U);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    portENTER_CRITICAL(&s_spinlock);
    g_current_brightness = brightness;
    portEXIT_CRITICAL(&s_spinlock);

    ESP_LOGD(TAG, "Brilho: %d%% (duty=%lu)", brightness, duty);
}

void dali_turn_on(void)    { dali_set_brightness(LIGHT_MAX); }
void dali_turn_off(void)   { dali_set_brightness(LIGHT_MIN); }
void dali_safe_mode(void)  { dali_set_brightness(LIGHT_SAFE_MODE); }

uint8_t dali_get_brightness(void)
{
    uint8_t value;
    portENTER_CRITICAL(&s_spinlock);
    value = g_current_brightness;
    portEXIT_CRITICAL(&s_spinlock);
    return value;
}
