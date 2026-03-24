/* ============================================================
   ST7789 — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      st7789.c
   @brief     Driver SPI para display TFT ST7789 240x240
   @version   4.0
   @date      2026-03-23

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Descrição:
   ----------
   Driver SPI completo para o controlador ST7789.
   Compatível com LVGL (flush via draw_bitmap).

   Características:
   ----------------
   - SPI HSPI (SPI2_HOST)
   - RGB565
   - Suporte a LVGL
   - Backlight controlado por GPIO
   - Configuração de cor ajustável (MADCTL)

============================================================ */

#include "st7789.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ST7789";

/* Handle SPI */
static spi_device_handle_t spi;

/* ============================================================
   CONTROLO DC
============================================================ */
static inline void dc_cmd(void)
{
    gpio_set_level(LCD_PIN_DC, 0);
}

static inline void dc_data(void)
{
    gpio_set_level(LCD_PIN_DC, 1);
}

/* ============================================================
   ENVIO SPI
============================================================ */
static void write_cmd(uint8_t cmd)
{
    spi_transaction_t t = {0};

    dc_cmd();

    t.length = 8;
    t.tx_buffer = &cmd;

    spi_device_transmit(spi, &t);
}

static void write_data(const uint8_t *data, int len)
{
    if (len == 0) return;

    spi_transaction_t t = {0};

    dc_data();

    t.length = len * 8;
    t.tx_buffer = data;

    spi_device_transmit(spi, &t);
}

/* ============================================================
   RESET HARDWARE
============================================================ */
static void reset_display(void)
{
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ============================================================
   INICIALIZAÇÃO
============================================================ */
void st7789_init(void)
{
    ESP_LOGI(TAG, "Inicializar ST7789...");

    /* --- Configuração GPIO --- */
    gpio_set_direction(LCD_PIN_DC,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_BL,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_CS,  GPIO_MODE_OUTPUT);

    /* Backlight OFF inicialmente */
    gpio_set_level(LCD_PIN_BL, 0);

    /* Reset hardware */
    reset_display();

    /* --- Configuração SPI --- */
    spi_bus_config_t buscfg = {
        .mosi_io_num     = LCD_PIN_SDA,
        .miso_io_num     = -1,
        .sclk_io_num     = LCD_PIN_SCL,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * 40 * 2
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20000000,
        .mode           = 0,
        .spics_io_num   = LCD_PIN_CS,
        .queue_size     = 7,
        .flags          = SPI_DEVICE_HALFDUPLEX
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    /* =========================================================
       SEQUÊNCIA ST7789
    ========================================================= */

    /* Sleep out */
    write_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* RGB565 */
    write_cmd(0x3A);
    uint8_t colmod = 0x55;
    write_data(&colmod, 1);

    /* MADCTL — TESTAR SE NECESSÁRIO:
       0x00 → RGB normal
       0x08 → BGR (se cores trocadas) */
    write_cmd(0x36);
    uint8_t madctl = 0x00;
    write_data(&madctl, 1);

    /* Inversão (comentar se cores estranhas) */
    write_cmd(0x21);

    /* Display ON */
    write_cmd(0x29);

    /* Backlight ON */
    gpio_set_level(LCD_PIN_BL, 1);

    ESP_LOGI(TAG, "ST7789 pronto");
}

/* ============================================================
   JANELA DE ESCRITA
============================================================ */
void st7789_set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    /* Colunas */
    write_cmd(0x2A);
    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;
    write_data(data, 4);

    /* Linhas */
    write_cmd(0x2B);
    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;
    write_data(data, 4);

    /* RAM write */
    write_cmd(0x2C);
}

/* ============================================================
   DRAW BITMAP — LVGL
============================================================ */
void st7789_draw_bitmap(uint16_t x, uint16_t y,
                        uint16_t w, uint16_t h,
                        const uint16_t *data)
{
    if (!data || w == 0 || h == 0) return;

    st7789_set_window(x, y, x + w - 1, y + h - 1);

    dc_data();

    spi_transaction_t t = {0};
    t.length    = w * h * 16;
    t.tx_buffer = data;

    spi_device_transmit(spi, &t);
}

/* ============================================================
   BACKLIGHT
============================================================ */
void st7789_backlight(bool on)
{
    gpio_set_level(LCD_PIN_BL, on ? 1 : 0);
}