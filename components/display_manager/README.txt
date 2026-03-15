DISPLAY MANAGER - ESP32 + ST7789
================================

Objetivo:
---------
Inicializar o display TFT ST7789 240x240 e permitir teste de mensagens.

Ligações:
----------
ESP32 → ST7789 (ordem do TFT: GRD, VCC, SCL, SDA, RST, DC, CS, BL)

GND      -> GRD
3.3V     -> VCC
GPIO18   -> SCL
GPIO23   -> SDA
GPIO33   -> RST
GPIO32   -> DC
GPIO5    -> CS
GPIO25   -> BL

Instruções de teste:
-------------------
1. Ligar o display conforme o esquema acima.
2. Compilar o projeto com ESP-IDF.
3. Executar main.c de teste para verificar mensagens no Serial Monitor.
4. Futuramente, integrar desenho de texto no display.

Observações:
------------
- O backlight (BL) deve estar ligado; caso contrário o ecrã ficará preto.
- Alimentação: 3.3V.
- Display: 240x240 pixels.