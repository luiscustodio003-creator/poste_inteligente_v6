/* ============================================================
   HARDWARE CONFIG — PINOS E PERIFÉRICOS
   ------------------------------------------------------------
   @file      hw_config.h
   @brief     Mapeamento de pinos GPIO e configuração de periféricos
   @version   2.0
   @date      2026-03-15

   Projecto  : Poste Inteligente
   Plataforma: ESP32 (ESP-IDF)

   Alterações (v1.0 → v2.0):
   --------------------------
   1. Adicionados pinos do radar HLK-LD2450 (UART)
   2. Adicionado pino PWM da luminária (LEDC)
   3. Adicionada configuração UART do radar

   Dependências:
   -------------
   - Nenhuma (apenas defines de pré-processador)
============================================================ */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* ============================================================
   DISPLAY TFT ST7789 — SPI
============================================================ */
#define LCD_PIN_SDA     23      /* MOSI                          */
#define LCD_PIN_SCL     18      /* SCLK                          */
#define LCD_PIN_CS       5      /* Chip Select                   */
#define LCD_PIN_DC      32      /* Data/Command                  */
#define LCD_PIN_RST     33      /* Reset                         */
#define LCD_PIN_BL      25      /* Backlight                     */

/* Resolução do ecrã — deve coincidir com system_config.h */
#define LCD_H_RES       240
#define LCD_V_RES       240

/* ============================================================
   RADAR HLK-LD2450 — UART
============================================================ */
#define RADAR_UART_PORT     UART_NUM_2  /* UART2 do ESP32          */
#define RADAR_PIN_TX        27         /* TX do ESP32 → RX radar  */
#define RADAR_PIN_RX        22          /* RX do ESP32 ← TX radar  */
#define RADAR_BAUD_RATE     115200      /* Baud rate do HLK-LD2450 */

/* ============================================================
   LUMINÁRIA — PWM (LEDC)
============================================================ */
#define LED_PWM_PIN         26          /* Pino PWM para controlo DALI/dimmer */

#endif /* HW_CONFIG_H */