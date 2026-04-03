/* ============================================================
   MAIN — PONTO DE ENTRADA v3.8
   ------------------------------------------------------------
   @file      main.c
   @brief     Inicialização e ciclo principal do Poste Inteligente
   @version   3.8
   @date      2026-03-31

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Alterações v3.7 → v3.8:
   -------------------------
   CORRECÇÃO 1 — Ordem de inicialização corrigida.
     Na v3.7 o radar arrancava ANTES de esp_netif_init() e
     esp_event_loop_create_default(), o que criava uma
     inconsistência: os passos [2/6] e [3/6] do log referiam-se
     à rede e à configuração do poste, mas na prática o radar
     já tinha corrido antes do passo [1/6] terminar.
     Embora não causasse crash (o radar não depende da rede),
     a ordem era enganosa e dificultava o diagnóstico.

     Ordem correcta v3.8:
       [1] NVS Flash
       [2] Rede (netif + event loop) — base para tudo o resto
       [3] Configuração do poste (NVS → ID, nome)
       [4] Hardware: DALI + Radar (init + auto-detect + diagnóstico)
       [5] Display LVGL + FSM + fsm_task
       [6] WiFi STA

     O radar continua a arrancar ANTES do WiFi (correcto —
     são independentes), mas DEPOIS da rede de eventos estar
     inicializada, evitando qualquer dependência implícita.

   CORRECÇÃO 2 — Comentários e logs actualizados para reflectir
     a nova ordem de passos de forma exacta.

   Mantidas todas as funcionalidades de v3.7:
   - Radar e FSM arrancam antes do WiFi
   - Display thread-safe via fila (display_manager v5.x)
   - comm_init() só após WiFi ligado
   - lv_timer_handler() exclusivo da main_task
   - fsm_task com delay de 3500ms + radar_flush_rx()

   Sequência de arranque completa (main_task):
   --------------------------------------------
     [1] nvs_flash_init()
     [2] esp_netif_init() + esp_event_loop_create_default()
     [3] post_config_init()
     [4] dali_init() + radar_init() + auto_detect + diagnóstico
     [5] display_manager_init() + state_machine_init() + fsm_task
     [6] wifi_manager_init()
     [loop] display + comm

   Dependências directas:
   ----------------------
   - system_config.h  : USE_RADAR, POSTE_NAME, MAX_IP_LEN
   - hw_config.h      : RADAR_PIN_TX, RADAR_PIN_RX
   - post_config.h    : post_config_init()
   - wifi_manager.h   : wifi_manager_init(), wifi_manager_is_connected()
   - display_manager.h: display_manager_init(), _tick(), _task(), set_*()
   - radar_manager.h  : radar_init(), radar_auto_detect_baud(),
                        radar_diagnostic(), radar_get_status_str(),
                        radar_is_connected(), radar_manager_get_objects()
   - comm_manager.h   : comm_init(), comm_status_ok(), comm_is_master(),
                        comm_left_online(), comm_right_online()
   - udp_manager.h    : udp_manager_get_neighbors()
   - dali_manager.h   : dali_init(), dali_get_brightness()
   - state_machine.h  : state_machine_init(), state_machine_update(),
                        state_machine_get_state_name(), get_T(), get_Tc(),
                        get_last_speed(), sm_inject_test_car()
============================================================ */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_event.h"
#include <string.h>
#include <stdbool.h>

/* Inclusões do Projecto */
#include "system_config.h"
#include "hw_config.h"
#include "post_config.h"
#include "wifi_manager.h"
#include "display_manager.h"
#include "radar_manager.h"
#include "comm_manager.h"
#include "udp_manager.h"
#include "dali_manager.h"
#include "state_machine.h"

/* Etiqueta de log deste módulo */
static const char *TAG = "MAIN";

/* ============================================================
   FLAGS DE TESTE MANUAL
   Podem ser alteradas via debugger ou consola série em tempo
   real para injectar um carro simulado sem hardware.
     g_test_car       = true  → dispara sm_inject_test_car()
     g_test_car_speed = N     → velocidade do carro (km/h)
============================================================ */
volatile bool  g_test_car       = false;
volatile float g_test_car_speed = 50.0f;

/* ============================================================
   init_nvs
   ------------------------------------------------------------
   Inicializa a partição NVS da flash.
   Se a partição estiver corrompida ou com versão incompatível,
   apaga-a e reinicializa — garante arranque limpo após
   alterações de esquema de dados na NVS.
============================================================ */
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS corrompida ou versao incompativel — a apagar...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS inicializada com sucesso");
}

/* ============================================================
   _atualiza_radar_display
   ------------------------------------------------------------
   Actualiza o canvas do radar no display_manager.

   Em modo UART (USE_RADAR=1):
     Lê objectos com rasto da cache s_last_data do radar_manager
     via radar_manager_get_objects(). Não chama radar_read_data()
     directamente — isso consumiria o ring buffer que a FSM já
     leu neste ciclo, resultando sempre em count=0 no canvas.

   Em modo simulado (USE_RADAR=0):
     Obtém a posição actual do carro simulado via sim_get_objeto()
     e constrói um radar_obj_t mínimo para o canvas.
============================================================ */
static void _atualiza_radar_display(void)
{
#if USE_RADAR == 0
    /* Modo simulado: obtém posição do simulador */
    float x_mm = 0.0f, y_mm = 0.0f;
    if (sim_get_objeto(&x_mm, &y_mm))
    {
        radar_obj_t obj = {0};
        obj.x_mm = (int)x_mm;
        obj.y_mm = (int)y_mm;
        display_manager_set_radar(&obj, 1);
    }
    else
    {
        display_manager_set_radar(NULL, 0);
    }
#else
    /* Modo UART: lê objectos com rasto da cache do radar_manager */
    radar_obj_t objs[RADAR_MAX_OBJ];
    uint8_t count = radar_manager_get_objects(objs, RADAR_MAX_OBJ);
    display_manager_set_radar(count > 0 ? objs : NULL, count);
#endif
}

/* ============================================================
   fsm_task — Ciclo de Controlo Principal
   ------------------------------------------------------------
   Período: 100ms. Prioridade: 6 (acima da main_task em 5).
   Stack: 6144 bytes (suficiente para state_machine + radar).

   Criada em [5/6] da main_task, APÓS o auto-detect e o
   diagnóstico do radar terem terminado — não há concorrência
   no acesso à UART durante o arranque.

   Sequência de arranque da task:
     1. vTaskDelay(3500ms) — aguarda estabilização completa do
        HLK-LD2450. Cobre o período de arranque a frio do
        oscilador interno (~500ms) e a variante com antena
        externa (-A) que demora mais a estabilizar.
     2. radar_flush_rx() — descarta bytes acumulados na UART
        durante os 3500ms de espera (backlog de boot).
     3. Loop FSM a 100ms.

   Responsabilidades do loop:
     1. Injecção de carro de teste (via debugger/consola)
     2. Actualização da máquina de estados (radar + UDP)
     3. Sincronização do display (não-bloqueante via fila)
     4. Actualização do canvas do radar
============================================================ */
static void fsm_task(void *arg)
{
    ESP_LOGI(TAG,
             "FSM task iniciada | periodo=100ms | aguarda 3500ms (radar)");

    /* Aguarda estabilização completa do HLK-LD2450.
     * 3500ms = margem generosa para arranque a frio e variante -A.
     * O auto-detect e o diagnóstico já correram na main_task antes
     * desta task ser criada — este delay é apenas para estabilização
     * final do sensor antes da primeira leitura real da FSM. */
    vTaskDelay(pdMS_TO_TICKS(6000));
    radar_flush_rx();

    while (1)
    {
        /* 1. Injecção de carro de teste manual (debugger/consola) */
        if (g_test_car)
        {
            g_test_car = false;
            sm_inject_test_car(g_test_car_speed);
        }

        /* 2. Actualiza lógica da máquina de estados.
         *    comm_status_ok() e comm_is_master() são thread-safe
         *    (lêem flags atómicas do comm_manager). */
        state_machine_update(comm_status_ok(), comm_is_master());

        /* 3. Sincroniza interface com a FSM.
         *    Todas as chamadas set_*() são não-bloqueantes —
         *    enfileiram mensagens via xQueueSend().
         *    O LVGL só é tocado em display_manager_task()
         *    na main_task (única task que chama LVGL). */
        display_manager_set_status(state_machine_get_state_name());
        display_manager_set_traffic(state_machine_get_T(),
                                    state_machine_get_Tc());
        display_manager_set_speed((int)state_machine_get_last_speed());
        display_manager_set_hardware(radar_get_status_str(),
                                     radar_is_connected(),
                                     (uint8_t)dali_get_brightness());

        /* 4. Actualiza canvas do radar no display */
        _atualiza_radar_display();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   main_task — Supervisão, LVGL e Rede
   ------------------------------------------------------------
   Prioridade 5. Única task que chama lv_timer_handler().
   Período do loop principal: 20ms (50 Hz — fluido para LVGL).

   Ordem de inicialização (v3.8 — corrigida):
   -------------------------------------------
   [1] NVS Flash
       Base de toda a configuração persistente. Deve ser o
       primeiro passo — post_config_init() depende disto.

   [2] Rede (netif + event loop)
       esp_netif_init() e esp_event_loop_create_default() devem
       ser chamados antes de qualquer driver que registe eventos
       (wifi_manager, etc.). Colocados aqui garante que a
       infraestrutura de eventos está pronta antes de ser usada.

   [3] Configuração do poste (NVS → ID, nome)
       Lê POSTE_ID e POSTE_NAME da NVS (ou usa defaults de
       system_config.h no primeiro arranque). Depende de [1].

   [4] Hardware: DALI + Radar
       DALI inicializa o PWM da luminária.
       Radar inicializa UART2, corre auto-detect de baud rate
       (até 3s) e abre janela de diagnóstico (8s).
       O radar arranca ANTES do WiFi porque são independentes
       e o sensor demora ~1s a estabilizar — iniciá-lo cedo
       maximiza o tempo disponível antes da fsm_task ler dados.

   [5] Display LVGL + FSM + fsm_task
       Display inicializado após hardware para mostrar estado
       correcto desde o primeiro frame.
       fsm_task criada aqui — começa com delay de 3500ms
       interno para aguardar estabilização final do sensor.

   [6] WiFi STA
       Último a inicializar — não é necessário para o controlo
       local (radar + luminária). comm_init() é chamado no loop
       principal apenas quando o WiFi estiver ligado.

   Loop principal (após [6]):
   --------------------------
   - Gere o display LVGL (única task a chamar lv_timer_handler)
   - Inicia comm_init() quando WiFi ficar disponível
   - Actualiza vizinhos e estado de rede no display
============================================================ */
static void main_task(void *arg)
{
    /* ----------------------------------------------------------
       [1/6] NVS Flash
       Base de toda a configuração persistente.
    ---------------------------------------------------------- */
    ESP_LOGI(TAG, "[1/6] Inicializando NVS...");
    init_nvs();

    /* ----------------------------------------------------------
       [2/6] Infraestrutura de rede e eventos
       Deve estar pronta antes de qualquer driver que registe
       handlers de eventos (WiFi, IP, etc.).
    ---------------------------------------------------------- */
    ESP_LOGI(TAG, "[2/6] Inicializando infraestrutura de rede...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ----------------------------------------------------------
       [3/6] Configuração do poste
       Carrega ID e nome da NVS. Depende de [1].
    ---------------------------------------------------------- */
    ESP_LOGI(TAG, "[3/6] Lendo configuracao do poste...");
    post_config_init();

    /* ----------------------------------------------------------
       [4/6] Hardware: DALI + Radar
       DALI: PWM da luminária — arranca em LIGHT_MIN.
       Radar: UART2 + auto-detect de baud + diagnóstico 8s.
       O radar arranca ANTES do WiFi (são independentes) para
       maximizar o tempo de estabilização do sensor.
    ---------------------------------------------------------- */
    ESP_LOGI(TAG, "[4/6] Inicializando hardware (DALI + Radar)...");

    dali_init();

#if USE_RADAR
    radar_init(RADAR_MODE_UART);

    int baud_det = radar_auto_detect_baud();
    if (baud_det > 0)
    {
        ESP_LOGI(TAG, "Radar baud detectado: %d", baud_det);
    }
    else
    {
        ESP_LOGW(TAG, "Radar baud nao detectado — verificar ligacoes");
        ESP_LOGW(TAG,"Ligacao correcta: TX sensor->GPIO%d | RX sensor->GPIO%d",RADAR_PIN_RX, RADAR_PIN_TX);
        ESP_LOGW(TAG,"Se invertido: trocar os fios TX<->RX fisicamente");
    }

    /* Janela de diagnóstico: 8s para confirmar funcionamento.
     * Passar a mão em frente ao sensor durante este período
     * para verificar detecção no monitor série. */
    //radar_diagnostic();

#else
    radar_init(RADAR_MODE_SIMULATED);
    ESP_LOGI(TAG, "Radar: modo simulado (USE_RADAR=0)");
#endif

    /* ----------------------------------------------------------
       [5/6] Display LVGL + FSM + fsm_task
       Display inicializado após hardware para mostrar estado
       correcto desde o arranque.
       fsm_task criada aqui — começa com delay interno de 3500ms
       para aguardar estabilização final do HLK-LD2450.
    ---------------------------------------------------------- */
    ESP_LOGI(TAG, "[5/6] Inicializando Display e FSM...");
    display_manager_init();
    state_machine_init();

    xTaskCreate(fsm_task, "fsm_task", 6144, NULL, 6, NULL);
    ESP_LOGI(TAG, "fsm_task criada (aguarda 3500ms interno)");

    /* ----------------------------------------------------------
       [6/6] WiFi STA
       Último a inicializar — o controlo local (radar + DALI)
       já está operacional antes do WiFi ficar ligado.
       comm_init() é chamado no loop principal apenas quando
       o WiFi confirmar ligação com IP obtido.
    ---------------------------------------------------------- */
    ESP_LOGI(TAG, "[6/6] Inicializando WiFi (STA)...");
    wifi_manager_init();

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  SISTEMA OPERACIONAL — Poste %s", POSTE_NAME);
    ESP_LOGI(TAG, "==============================================");

    /* ----------------------------------------------------------
       LOOP PRINCIPAL
       Período: 20ms (50 Hz — fluido para o LVGL).

       Responsabilidades:
         - Gerir o display LVGL (única task que chama LVGL)
         - Iniciar comm_init() quando WiFi ficar disponível
         - Actualizar vizinhos e estado de rede no display
    ---------------------------------------------------------- */
    bool     comm_iniciado = false;
    uint32_t last_tick     = (uint32_t)(esp_timer_get_time() / 1000);

    while (1)
    {
        /* ── Display LVGL (única task autorizada a chamar LVGL) ── */
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        display_manager_tick(now - last_tick);
        display_manager_task();   /* drena fila + lv_timer_handler() */
        last_tick = now;

        /* ── Comunicação entre postes ── */
        bool wifi_curr = wifi_manager_is_connected();

        /* Inicializa UDP apenas uma vez, após WiFi ligado com IP */
        if (!comm_iniciado && wifi_curr)
        {
            if (comm_init())
            {
                comm_iniciado = true;
                ESP_LOGI(TAG, "Protocolo UDP iniciado — postes a descobrir vizinhos");
            }
        }

        /* Actualiza display com estado de rede */
        if (comm_iniciado)
        {
            char nL[MAX_IP_LEN], nR[MAX_IP_LEN];
            udp_manager_get_neighbors(nL, nR);
            display_manager_set_neighbors(nL, nR,
                                          comm_left_online(),
                                          comm_right_online());
            display_manager_set_wifi(true, wifi_manager_get_ip());
        }
        else
        {
            display_manager_set_wifi(false, "Desligado");
        }

        /* 20ms → LVGL fluido a 50 Hz */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
   app_main — Ponto de entrada do ESP-IDF
   ------------------------------------------------------------
   Cria a main_task com stack de 8192 bytes.
   Stack generosa para suportar: WiFi stack + LVGL heap init
   + inicialização da NVS + chamadas ESP-IDF de arranque.
   Prioridade 5 — abaixo da fsm_task (6) para ceder CPU
   durante o ciclo de controlo do radar e luminária.
============================================================ */
void app_main(void)
{
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}