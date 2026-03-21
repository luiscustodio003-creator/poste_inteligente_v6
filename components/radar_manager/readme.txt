RADAR MANAGER - HLK-LD2450
--------------------------

Projeto: Poste Inteligente
Plataforma: ESP32 (ESP-IDF)
Radar: HLK-LD2450 (24GHz)

Descrição:
- Modulo responsável por inicializar e ler o radar via UART.
- Suporta múltiplos alvos (até 8) com distância (m) e velocidade (km/h).
- Modo simulado disponível para testes de bancada.
- Integrado com FSM (state_machine) e dali_manager (LED).

Pinos ESP32:
- TX Radar -> RX ESP32 (GPIO16)
- RX Radar -> TX ESP32 (GPIO17) [opcional]
- Alimentação 5V e GND

Notas:
- HLK-LD2450 envia pacotes UART contendo distância e velocidade.
- A leitura do radar retorna radar_data_t com todos os alvos detectados.
- Modo SAFE é acionado se o radar não responder.