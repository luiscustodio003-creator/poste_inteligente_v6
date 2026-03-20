/* ============================================================
   DRIVER ST7789 — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      st7789.c
   @brief     Driver SPI para display TFT ST7789 240x240
   @version   2.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)


   Dependências:
   -------------
   - hw_config.h  (pinos GPIO e resolução)
   - ESP-IDF: driver/spi_master, driver/gpio, freertos
============================================================ */

#include "st7789.h"
#include "hw_config.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "ST7789";

/* Handle do dispositivo SPI */
static spi_device_handle_t spi;

/* -----------------------------------------------------------
   Envia um byte de comando ao controlador ST7789.
   DC = 0 indica modo comando.
   ----------------------------------------------------------- */
static void cmd(uint8_t c)
{
    gpio_set_level(LCD_PIN_DC, 0);  /* DC baixo = comando */

    spi_transaction_t t = {0};
    t.length    = 8;
    t.tx_buffer = &c;

    spi_device_transmit(spi, &t);
}

/* -----------------------------------------------------------
   Envia dados ao controlador ST7789.
   DC = 1 indica modo dados.
   ----------------------------------------------------------- */
static void data(const void *data, int len)
{
    gpio_set_level(LCD_PIN_DC, 1);  /* DC alto = dados */

    spi_transaction_t t = {0};
    t.length    = len * 8;
    t.tx_buffer = data;

    spi_device_transmit(spi, &t);
}

/* -----------------------------------------------------------
   Define a janela de endereçamento activa (CASET + RASET).
   Todos os desenhos subsequentes ocorrem nesta região.
   x0,y0 — canto superior esquerdo
   x1,y1 — canto inferior direito (inclusivo)
   ----------------------------------------------------------- */
static void set_window(int x0, int y0, int x1, int y1)
{
    uint8_t d[4];

    /* CASET — Column Address Set */
    cmd(0x2A);
    d[0] = x0 >> 8;
    d[1] = x0 & 0xFF;
    d[2] = x1 >> 8;
    d[3] = x1 & 0xFF;
    data(d, 4);

    /* RASET — Row Address Set */
    cmd(0x2B);
    d[0] = y0 >> 8;
    d[1] = y0 & 0xFF;
    d[2] = y1 >> 8;
    d[3] = y1 & 0xFF;
    data(d, 4);

    /* RAMWR — Memory Write (inicia escrita de pixels) */
    cmd(0x2C);
}

/* -----------------------------------------------------------
   Desenha um bitmap RGB565 numa posição e dimensão dadas.
   x,y   — canto superior esquerdo
   w,h   — largura e altura em pixels
   data_ptr — array de pixels no formato RGB565 (big-endian)
   ----------------------------------------------------------- */
void st7789_draw_bitmap(int x, int y, int w, int h, const uint16_t *data_ptr)
{
    set_window(x, y, x + w - 1, y + h - 1);
    data(data_ptr, w * h * 2);  /* 2 bytes por pixel (RGB565) */
}

/* -----------------------------------------------------------
   Preenche o ecrã inteiro com uma cor sólida.
   color — cor no formato RGB565
   ----------------------------------------------------------- */
void st7789_fill(uint16_t color)
{
    static uint16_t line[LCD_H_RES];

    /* Preenche o buffer de linha com a cor desejada */
    for (int i = 0; i < LCD_H_RES; i++)
        line[i] = color;

    /* Envia linha a linha para o display */
    for (int y = 0; y < LCD_V_RES; y++)
        st7789_draw_bitmap(0, y, LCD_H_RES, 1, line);
}

/* -----------------------------------------------------------
   Inicializa o display ST7789.

   Sequência de inicialização (datasheet ST7789V2):
     1. Reset hardware
     2. SLPOUT — sai do modo sleep
     3. COLMOD — define formato de cor (RGB565)
     4. MADCTL — define ordem de memória e ordem de cor (RGB)
     5. INVON  — inversão de cor (necessária na maioria dos
                  módulos comerciais 240x240)
     6. DISPON — liga o display
   ----------------------------------------------------------- */
void st7789_init(void)
{
    ESP_LOGI(TAG, "Inicializar ST7789");

    /* Configura pinos de controlo como saída */
    gpio_set_direction(LCD_PIN_DC,  GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_BL,  GPIO_MODE_OUTPUT);

    /* Reset hardware: pulso baixo de 50 ms */
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));  /* Aguarda estabilização após reset */

    /* Inicializa barramento SPI */
    spi_bus_config_t buscfg = {
        .mosi_io_num   = LCD_PIN_SDA,
        .miso_io_num   = -1,            /* MISO não utilizado */
        .sclk_io_num   = LCD_PIN_SCL,
        .max_transfer_sz = LCD_H_RES * 40 * 2
    };
    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    /* Configura dispositivo SPI */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20000000,         /* 20 MHz */
        .mode           = 0,                /* SPI modo 0 (CPOL=0, CPHA=0) */
        .spics_io_num   = LCD_PIN_CS,
        .queue_size     = 7,
        .flags          = SPI_DEVICE_HALFDUPLEX
    };
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi);

    /* Liga retroiluminação */
    gpio_set_level(LCD_PIN_BL, 1);

    /* --- Sequência de inicialização do ST7789 --- */

    /* 1. SLPOUT — sai do modo sleep, aguarda 120 ms obrigatórios */
    cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* 2. COLMOD — formato de cor: 0x55 = RGB565 (16 bits por pixel) */
    cmd(0x3A);
    uint8_t colmod = 0x55;
    data(&colmod, 1);

    /* 3. MADCTL — Memory Access Control
       CORRECÇÃO: Este comando define a ordem dos bytes de cor.
       0x00 = ordem RGB normal (sem espelho, sem rotação, RGB)
       Se as cores ainda aparecerem invertidas, experimenta 0x08
       (activa o bit BGR). Depende do módulo físico utilizado.    */
    cmd(0x36);
    uint8_t madctl = 0x00;
    data(&madctl, 1);

    /* 4. INVON — Inversão de cor activa
       A maioria dos módulos comerciais 240x240 com ST7789 precisa
       desta inversão para as cores aparecerem correctas.
       Se as cores ficarem negativas/invertidas, comenta esta linha. */
    cmd(0x21);

    /* 5. DISPON — liga o display */
    cmd(0x29);

    ESP_LOGI(TAG, "Display ST7789 pronto");
}