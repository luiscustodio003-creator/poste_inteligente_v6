/* ============================================================
   UDP MANAGER — IMPLEMENTAÇÃO
   ------------------------------------------------------------
   @file      udp_manager.c
   @brief     Comunicação UDP entre postes — protocolo completo
   @version   3.4
   @date      2026-04-01

   Projecto  : Poste Inteligente
   Estudantes: Luis Custodio | Tiago Moreno
   Plataforma: ESP32 (ESP-IDF)

   Alterações v3.3 → v3.4:
   ------------------------
   CORRECÇÃO 5 — Spam de logs no monitor série eliminado.
     Problema: a task UDP corre a cada 10ms (SO_RCVTIMEO=10ms).
     Vários ESP_LOGI repetiam-se centenas de vezes por segundo:
       - DISCOVER enviado periodicamente aparecia sempre no monitor
         sem valor de diagnóstico adicional em operação normal.
       - _encontrar_ou_criar_vizinho() imprimia "Novo vizinho"
         mesmo quando o vizinho já existia e só actualizava campos.
       - DISCOVER recebido de vizinho conhecido era LOGI a cada
         DISCOVER_INTERVAL_MS.
       - SPD, TC_INC e TC_INC- eram LOGI — com tráfego activo
         inundavam completamente o monitor.

     Solução — política de níveis de log clara:
       ESP_LOGI → eventos que ocorrem uma vez ou raramente:
                  novo vizinho (primeira vez), vizinho OFFLINE,
                  init, primeiro DISCOVER no arranque.
       ESP_LOGD → eventos periódicos ou de ciclo normal:
                  DISCOVER periódico enviado/recebido,
                  SPD recebido, TC_INC recebido.
       ESP_LOGW → condições anómalas: timeout vizinho, falha envio.
       ESP_LOGE → erros críticos: falha socket, bind.

     Para ver logs de ciclo: idf.py monitor --log-level DEBUG
     Em operação normal (INFO): apenas eventos de estado.

   CORRECÇÃO 6 — Comentários de protocolo em _processar_mensagem().
     Cada bloco de mensagem tem comentário com formato, campos
     esperados e acções tomadas pelo handler.

   Alterações v3.2 → v3.3 (mantidas):
   ------------------------------------
   CORRECÇÃO 4: SO_RCVTIMEO e vTaskDelay reduzidos de 100ms para 10ms.
   CORRECÇÃO 1: STATUS recebido cria vizinho se não existir.
   CORRECÇÃO 2: DISCOVER enviado imediatamente no arranque.
   CORRECÇÃO 3: udp_manager_get_neighbors() usa position.

============================================================ */

#include "udp_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "UDP_MGR";

/* ============================================================
   ESTADO INTERNO
============================================================ */
static int        s_socket      = -1;
static bool       s_iniciado    = false;
static neighbor_t s_vizinhos[MAX_NEIGHBORS];

/* CORRECÇÃO 2: inicializado a 0 menos o intervalo para enviar
   DISCOVER imediatamente na primeira iteração da task */
static uint32_t   s_ultimo_disc = 0;

/* ============================================================
   UTILITÁRIOS
============================================================ */

static uint32_t _agora_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char *_status_str(neighbor_status_t s)
{
    switch (s)
    {
        case NEIGHBOR_OK:        return "OK";
        case NEIGHBOR_OFFLINE:   return "OFFLINE";
        case NEIGHBOR_FAIL:      return "FAIL";
        case NEIGHBOR_SAFE:      return "SAFE";
        case NEIGHBOR_AUTO:      return "AUTO";
        case NEIGHBOR_OBSTACULO: return "OBST";   /* v3.1 */
        default:                 return "?";
    }
}

static neighbor_status_t _str_to_status(const char *s)
{
    if (strcmp(s, "OK")   == 0) return NEIGHBOR_OK;
    if (strcmp(s, "FAIL") == 0) return NEIGHBOR_FAIL;
    if (strcmp(s, "SAFE") == 0) return NEIGHBOR_SAFE;
    if (strcmp(s, "AUTO") == 0) return NEIGHBOR_AUTO;
    if (strcmp(s, "OBST") == 0) return NEIGHBOR_OBSTACULO;  /* v3.1 */
    return NEIGHBOR_OFFLINE;
}

/* ============================================================
   _enviar_para
   ------------------------------------------------------------
   Envia uma mensagem UDP para o IP e porto UDP_PORT indicados.
   Função interna — chamada por todas as funções de envio.
   Falhas de envio (ex: rede em baixo) são registadas como LOGW.
   Envios bem-sucedidos apenas em LOGD (alta frequência).
============================================================ */
static bool _enviar_para(const char *ip, const char *msg)
{
    if (s_socket < 0 || !ip || !msg) return false;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    dest.sin_addr.s_addr = inet_addr(ip);

    int r = sendto(s_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));

    if (r < 0)
    {
        /* LOGW: falha de envio — condição anómala, pouca frequência */
        ESP_LOGW(TAG, "Falha envio para %s: %s (errno=%d)",
                 ip, msg, errno);
        return false;
    }

    /* LOGD: envio normal — alta frequência, só visível em DEBUG */
    ESP_LOGD(TAG, "-> %s : %s", ip, msg);
    return true;
}

/* ============================================================
   _encontrar_ou_criar_vizinho
   ------------------------------------------------------------
   Procura vizinho existente pelo IP. Se não existir, cria entrada
   nova na tabela s_vizinhos[].

   Log:
     LOGI → novo vizinho criado (primeira vez — evento raro)
     LOGD → vizinho actualizado (evento de ciclo — alta frequência)
     LOGW → tabela cheia (MAX_NEIGHBORS atingido)
============================================================ */
static neighbor_t *_encontrar_ou_criar_vizinho(const char *ip,
                                                int         id,
                                                int         position)
{
    /* Procura entrada existente pelo IP */
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_vizinhos[i].active &&
            strcmp(s_vizinhos[i].ip, ip) == 0)
        {
            /* Actualiza campos se vierem mais precisos */
            if (id >= 0)       s_vizinhos[i].id       = id;
            if (position >= 0) s_vizinhos[i].position = position;
            /* LOGD: actualização periódica — não inunda o monitor */
            ESP_LOGD(TAG, "Vizinho actualizado ID=%d pos=%d IP=%s",
                     id, position, ip);
            return &s_vizinhos[i];
        }
    }

    /* Procura entrada livre para novo vizinho */
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_vizinhos[i].active)
        {
            memset(&s_vizinhos[i], 0, sizeof(neighbor_t));
            strncpy(s_vizinhos[i].ip, ip,
                    sizeof(s_vizinhos[i].ip) - 1);
            s_vizinhos[i].id       = id;
            s_vizinhos[i].position = position;
            s_vizinhos[i].status   = NEIGHBOR_OK;
            s_vizinhos[i].active   = true;

            /* LOGI: novo vizinho — evento raro, sempre visível */
            ESP_LOGI(TAG, "Novo vizinho registado: ID=%d pos=%d IP=%s",
                     id, position, ip);
            return &s_vizinhos[i];
        }
    }

    ESP_LOGW(TAG, "Tabela de vizinhos cheia (MAX=%d) — ignorar IP=%s",
             MAX_NEIGHBORS, ip);
    return NULL;
}

/* ============================================================
   _processar_mensagem
   ------------------------------------------------------------
   Descodifica e processa uma mensagem UDP recebida.
   Cada bloco trata um tipo de mensagem do protocolo.
   Chamado pela udp_task a cada pacote recebido.
============================================================ */
static void _processar_mensagem(char *msg, const char *ip_origem)
{
    if (!msg || !ip_origem) return;

    /* ----------------------------------------------------------
       DISCOVER:<id>:<position>
       ----------------------------------------------------------
       Broadcast periódico de presença enviado por todos os postes.
       Permite descoberta automática de vizinhos sem configuração
       manual de IPs.

       Acções:
         1. Ignora se vier de nós próprios (id == POSTE_ID)
         2. Regista/actualiza vizinho com IP, id e position
         3. Marca discover_ok=true — activa vigilância de timeout
         4. Responde com DISCOVER próprio para garantir registo mútuo

       Log:
         LOGI → primeira vez que este vizinho é visto (novo registo)
         LOGD → DISCOVER periódico de vizinho já conhecido
    ---------------------------------------------------------- */
    if (strncmp(msg, "DISCOVER:", 9) == 0)
    {
        char *p  = msg + 9;
        int   id = atoi(p);
        if (id == POSTE_ID) return;  /* ignorar eco próprio */

        /* Extrai posição — fallback para id se campo ausente */
        int   pos = id;
        char *sep = strchr(p, ':');
        if (sep) pos = atoi(sep + 1);

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip_origem,
                                                     id, pos);
        if (v)
        {
            bool era_novo = !v->discover_ok;
            v->last_seen    = _agora_ms();
            v->status       = NEIGHBOR_OK;
            v->active       = true;
            v->discover_ok  = true;

            if (era_novo)
            {
                /* LOGI: primeiro DISCOVER deste vizinho — evento raro */
                ESP_LOGI(TAG, "Vizinho descoberto: ID=%d pos=%d IP=%s [%s]",
                         id, pos, ip_origem,
                         pos < POST_POSITION ? "ESQ" : "DIR");
            }
            else
            {
                /* LOGD: heartbeat periódico — não inunda o monitor */
                ESP_LOGD(TAG, "DISCOVER heartbeat ID=%d [%s]",
                         id, pos < POST_POSITION ? "ESQ" : "DIR");
            }
        }

        /* Responde com DISCOVER próprio para garantir registo mútuo.
         * O vizinho que acabou de enviar também precisa de nos registar. */
        char resp[48];
        snprintf(resp, sizeof(resp), "DISCOVER:%d:%d",
                 POSTE_ID, POST_POSITION);
        _enviar_para(ip_origem, resp);
        return;
    }

    /* ----------------------------------------------------------
       STATUS:<id>:<estado>
       ----------------------------------------------------------
       Propaga estado operacional do poste: "OK","FAIL","SAFE","AUTO".
       Usado para detectar falhas e activar modo autónomo na FSM.

       Acções:
         1. Ignora se vier de nós próprios
         2. Cria vizinho se não existe (pode ser o primeiro contacto)
            Nota: discover_ok fica false até DISCOVER ser recebido —
            o timeout de vizinho não actua enquanto discover_ok=false.
         3. Actualiza status do vizinho e timestamp last_seen

       Log:
         LOGI → mudança de estado (ex: OK→OFFLINE, FAIL→OK)
         LOGD → confirmação de estado sem mudança
    ---------------------------------------------------------- */
    if (strncmp(msg, "STATUS:", 7) == 0)
    {
        char *p  = msg + 7;
        int   id = atoi(p);
        if (id == POSTE_ID) return;

        p = strchr(p, ':'); if (!p) return; p++;
        neighbor_status_t novo_status = _str_to_status(p);

        neighbor_t *v = _encontrar_ou_criar_vizinho(ip_origem,
                                                     id, id);
        if (v)
        {
            neighbor_status_t ant = v->status;
            v->status             = novo_status;
            v->last_seen          = _agora_ms();

            if (ant != novo_status)
            {
                /* LOGI: mudança de estado — evento informativo */
                ESP_LOGI(TAG, "STATUS ID=%d: %s → %s",
                         id,
                         _status_str(ant),
                         _status_str(novo_status));
            }
            else
            {
                /* LOGD: confirmação de estado igual — periódico */
                ESP_LOGD(TAG, "STATUS ID=%d: %s (sem mudança)",
                         id, _status_str(novo_status));
            }
        }
        return;
    }

    /* ----------------------------------------------------------
       SPD:<id>:<vel>:<eta_ms>:<dist_m>
       ----------------------------------------------------------
       Enviado pelo poste anterior quando detecta carro.
       Fornece velocidade e ETA para o poste seguinte calcular
       quando deve acender antecipadamente (pré-iluminação).

       Campos:
         vel    — velocidade do carro em km/h
         eta_ms — tempo estimado até o carro chegar (ms)
         dist_m — distância entre postes (m) — informativo

       Acção: chama on_spd_received() → state_machine.c
              que agenda s_acender_em_ms = agora + eta_ms - MARGEM

       Log: LOGD — alta frequência durante tráfego activo
    ---------------------------------------------------------- */
    if (strncmp(msg, "SPD:", 4) == 0)
    {
        char    *p   = msg + 4;
        int      id  = atoi(p);
        if (id == POSTE_ID) return;

        float    vel  = 0.0f;
        uint32_t eta  = 0;
        uint32_t dist = 0;

        p = strchr(p, ':'); if (!p) return; p++;
        vel = strtof(p, NULL);
        p = strchr(p, ':'); if (!p) return; p++;
        eta = (uint32_t)atol(p);
        p = strchr(p, ':'); if (!p) return; p++;
        dist = (uint32_t)atol(p);
        (void)dist;  /* recebido no protocolo, ETA calculado localmente */

        /* LOGD: evento de ciclo de tráfego — alta frequência */
        ESP_LOGD(TAG, "SPD de ID=%d: %.1fkm/h ETA=%lums",
                 id, vel, (unsigned long)eta);
        on_spd_received(vel, eta);
        return;
    }

    /* ----------------------------------------------------------
       TC_INC:<id>:<vel>
       ----------------------------------------------------------
       Enviado pelo poste anterior para coordenar contadores T/Tc.

       Velocidade positiva (vel >= 0):
         → Carro a caminho deste poste: Tc++
         → Chama on_tc_inc_received(vel) → state_machine.c

       Velocidade negativa (vel < 0):
         → Carro confirmado no poste seguinte: T-- neste poste
         → Chama on_prev_passed_received() → state_machine.c

       Log: LOGD — pode ocorrer a alta frequência com tráfego
    ---------------------------------------------------------- */
    if (strncmp(msg, "TC_INC:", 7) == 0)
    {
        char *p  = msg + 7;
        int   id = atoi(p);
        if (id == POSTE_ID) return;

        p = strchr(p, ':'); if (!p) return; p++;
        float vel = strtof(p, NULL);

        if (vel >= 0.0f)
        {
            /* LOGD: TC_INC é enviado por cada detecção de carro */
            ESP_LOGD(TAG, "TC_INC+ ID=%d: %.1fkm/h (Tc++)", id, vel);
            on_tc_inc_received(vel);
        }
        else
        {
            /* LOGD: PASSED é enviado quando carro chega ao poste seguinte */
            ESP_LOGD(TAG, "TC_INC- ID=%d: carro passou (T--)", id);
            on_prev_passed_received();
        }
        return;
    }

    /* ----------------------------------------------------------
       MASTER_CLAIM:<id>
       ----------------------------------------------------------
       Enviado pelo poste com POST_POSITION=0 ao arrancar ou
       periodicamente (heartbeat a cada 30s) para garantir que
       toda a cadeia sabe quem é o líder.

       Acção: chama on_master_claim_received(id) → state_machine.c
              que cede liderança e propaga em cadeia (A→B→C→D).

       Log: LOGI — evento de configuração de cadeia (pouco frequente)
    ---------------------------------------------------------- */
    if (strncmp(msg, "MASTER_CLAIM:", 13) == 0)
    {
        int id = atoi(msg + 13);
        if (id == POSTE_ID) return;

        /* LOGI: evento de gestão de cadeia — pouco frequente */
        ESP_LOGI(TAG, "MASTER_CLAIM de ID=%d — cedemos liderança", id);
        on_master_claim_received(id);
        return;
    }

    /* Mensagem de protocolo desconhecido — ignorar silenciosamente
     * em INFO, registar em DEBUG para diagnóstico */
    ESP_LOGD(TAG, "Mensagem desconhecida de %s: %s", ip_origem, msg);
}

/* ============================================================
   _verificar_timeouts
   ------------------------------------------------------------
   Verifica se algum vizinho ultrapassou NEIGHBOR_TIMEOUT_MS
   sem enviar DISCOVER. Se sim, marca-o como OFFLINE.

   Regras:
     - Só verifica vizinhos com discover_ok=true. Vizinhos
       criados apenas por STATUS aguardam o primeiro DISCOVER
       para ter posição correcta — sem DISCOVER não entram
       em timeout.
     - A transição para OFFLINE é registada em LOGI (evento
       de estado importante, pouco frequente).
     - Verificação repetida de vizinho já OFFLINE é silenciosa
       (LOGD) para não inundar o monitor.

   Chamada a cada iteração da udp_task (~10ms).
   O custo é O(MAX_NEIGHBORS) = O(constante) — negligenciável.
============================================================ */
static void _verificar_timeouts(uint32_t agora)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_vizinhos[i].active)       continue;
        if (!s_vizinhos[i].discover_ok)  continue;

        uint32_t delta = agora - s_vizinhos[i].last_seen;

        if (delta > NEIGHBOR_TIMEOUT_MS &&
            s_vizinhos[i].status != NEIGHBOR_OFFLINE)
        {
            /* LOGI: transição para OFFLINE — evento de estado importante */
            ESP_LOGI(TAG,
                     "Vizinho ID=%d IP=%s → OFFLINE (%lums sem resposta)",
                     s_vizinhos[i].id,
                     s_vizinhos[i].ip,
                     (unsigned long)delta);
            s_vizinhos[i].status = NEIGHBOR_OFFLINE;
        }
    }
}

/* ============================================================
   udp_task
   ------------------------------------------------------------
   Task FreeRTOS da comunicação UDP. Prioridade 5, stack 4096.
   Ciclo a ~10ms (SO_RCVTIMEO=10ms + vTaskDelay=10ms).

   Responsabilidades por iteração:
     1. recvfrom() — aguarda pacote até 10ms (não bloqueante)
     2. _processar_mensagem() — se pacote recebido
     3. udp_manager_discover() — se DISCOVER_INTERVAL_MS expirou
     4. _verificar_timeouts() — marca vizinhos OFFLINE se silenciosos

   Nota sobre logs:
     DISCOVER periódico usa LOGD — não aparece em nível INFO.
     Para diagnóstico completo: idf.py monitor --log-level DEBUG
============================================================ */
static void udp_task(void *arg)
{
    char               rx_buf[128];
    struct sockaddr_in addr_origem;

    ESP_LOGI(TAG, "Task UDP iniciada (porto %d)", UDP_PORT);

    /* CORRECÇÃO 2: envia DISCOVER imediatamente no arranque
       para que os vizinhos nos descubram o mais rápido possível,
       sem esperar DISCOVER_INTERVAL_MS. Log LOGI apenas aqui. */
    udp_manager_discover();
    ESP_LOGI(TAG, "DISCOVER inicial enviado (arranque)");
    s_ultimo_disc = _agora_ms();

    while (1)
    {
        /* ── 1. Recebe pacote UDP (bloqueia até 10ms) ── */
        socklen_t addr_len = sizeof(addr_origem);
        int r = recvfrom(s_socket,
                         rx_buf, sizeof(rx_buf) - 1,
                         0,
                         (struct sockaddr *)&addr_origem,
                         &addr_len);

        if (r > 0)
        {
            rx_buf[r] = '\0';
            char ip_str[16];
            inet_ntoa_r(addr_origem.sin_addr,
                        ip_str, sizeof(ip_str));
            _processar_mensagem(rx_buf, ip_str);
        }

        uint32_t agora = _agora_ms();

        /* ── 2. DISCOVER periódico ── */
        if ((agora - s_ultimo_disc) >= DISCOVER_INTERVAL_MS)
        {
            udp_manager_discover();
            /* LOGD: evento periódico — não inunda o monitor em INFO */
            ESP_LOGD(TAG, "DISCOVER periódico enviado");
            s_ultimo_disc = agora;
        }

        /* ── 3. Verifica timeouts de vizinhos ── */
        _verificar_timeouts(agora);

        /* CORRECÇÃO v3.3: 10ms — consistente com SO_RCVTIMEO=10ms.
         * Garante que TC_INC e SPD enviados em sequência são
         * processados com diferença máxima de 10ms entre iterações. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
   udp_manager_init
============================================================ */
bool udp_manager_init(void)
{
    if (s_iniciado)
    {
        ESP_LOGW(TAG, "Já iniciado — ignorar");
        return true;
    }

    memset(s_vizinhos, 0, sizeof(s_vizinhos));
    s_ultimo_disc = 0;

    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket < 0)
    {
        ESP_LOGE(TAG, "Falha socket (errno=%d)", errno);
        return false;
    }

    int bc = 1;
    setsockopt(s_socket, SOL_SOCKET, SO_BROADCAST,
               &bc, sizeof(bc));

    /* CORRECÇÃO v3.3: 10ms em vez de 100ms — garante que TC_INC
       e SPD enviados em sequência são processados com diferença
       máxima de 10ms, sem atrasar o timer de pré-acendimento. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
    setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO,
               &tv, sizeof(tv));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_socket,
             (struct sockaddr *)&bind_addr,
             sizeof(bind_addr)) < 0)
    {
        ESP_LOGE(TAG, "Falha bind porto %d (errno=%d)",
                 UDP_PORT, errno);
        close(s_socket);
        s_socket = -1;
        return false;
    }

    xTaskCreate(udp_task, "udp_task", 4096, NULL, 5, NULL);

    s_iniciado = true;
    ESP_LOGI(TAG, "UDP Manager v3.4 pronto (porto %d)", UDP_PORT);
    return true;
}

/* ============================================================
   udp_manager_discover
   ------------------------------------------------------------
   Envia broadcast DISCOVER para toda a rede local.
   Formato: "DISCOVER:<id>:<position>"

   Chamada:
     - Uma vez no arranque da udp_task (LOGI nesse contexto)
     - Periodicamente a cada DISCOVER_INTERVAL_MS (LOGD aqui)
     - Pode ser chamada externamente pelo comm_manager

   Nota: o log aqui é LOGD porque esta função é chamada
   periodicamente — em INFO só o DISCOVER inicial é visível
   (registado na udp_task com LOGI).
============================================================ */
void udp_manager_discover(void)
{
    if (s_socket < 0) return;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT);
    dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    char msg[32];
    snprintf(msg, sizeof(msg),
             "DISCOVER:%d:%d", POSTE_ID, POST_POSITION);

    int r = sendto(s_socket, msg, strlen(msg), 0,
                   (struct sockaddr *)&dest, sizeof(dest));

    if (r < 0)
        ESP_LOGW(TAG, "Falha DISCOVER broadcast (errno=%d)", errno);
    else
        ESP_LOGD(TAG, "DISCOVER: %s", msg);
}

/* ============================================================
   Funções de envio
============================================================ */
bool udp_manager_send_spd(const char *ip, float speed,
                            uint32_t eta_ms, uint32_t dist_m)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "SPD:%d:%.1f:%lu:%lu",
             POSTE_ID, speed,
             (unsigned long)eta_ms, (unsigned long)dist_m);
    return _enviar_para(ip, msg);
}

bool udp_manager_send_tc_inc(const char *ip, float speed)
{
    char msg[32];
    snprintf(msg, sizeof(msg),
             "TC_INC:%d:%.1f", POSTE_ID, speed);
    return _enviar_para(ip, msg);
}

bool udp_manager_send_status(const char *ip,
                               neighbor_status_t status)
{
    char msg[32];
    snprintf(msg, sizeof(msg),
             "STATUS:%d:%s", POSTE_ID, _status_str(status));
    return _enviar_para(ip, msg);
}

bool udp_manager_send_master_claim(const char *ip)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "MASTER_CLAIM:%d", POSTE_ID);
    return _enviar_para(ip, msg);
}

/* ============================================================
   udp_manager_get_neighbors — compatibilidade display
============================================================ */
void udp_manager_get_neighbors(char *nebL, char *nebR)
{
    if (!nebL || !nebR) return;

    strncpy(nebL, "---", 16);
    strncpy(nebR, "---", 16);

    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (!s_vizinhos[i].active) continue;

        if (s_vizinhos[i].position < POST_POSITION)
        {
            strncpy(nebL, s_vizinhos[i].ip, 15);
            nebL[15] = '\0';
        }
        else if (s_vizinhos[i].position > POST_POSITION)
        {
            strncpy(nebR, s_vizinhos[i].ip, 15);
            nebR[15] = '\0';
        }
    }
}

/* ============================================================
   udp_manager_get_neighbor_by_pos
============================================================ */
neighbor_t *udp_manager_get_neighbor_by_pos(int position)
{
    for (int i = 0; i < MAX_NEIGHBORS; i++)
    {
        if (s_vizinhos[i].active &&
            s_vizinhos[i].position == position)
            return &s_vizinhos[i];
    }
    return NULL;
}

/* ============================================================
   udp_manager_get_all_neighbors
============================================================ */
size_t udp_manager_get_all_neighbors(neighbor_t *list, size_t max)
{
    if (!list || max == 0) return 0;
    size_t n = 0;
    for (int i = 0; i < MAX_NEIGHBORS && n < max; i++)
        if (s_vizinhos[i].active)
            list[n++] = s_vizinhos[i];
    return n;
}

/* ============================================================
   CALLBACKS WEAK
============================================================ */
__attribute__((weak))
void on_tc_inc_received(float speed)
{ ESP_LOGD(TAG, "on_tc_inc_received(%.1f) sem handler", speed); }

__attribute__((weak))
void on_prev_passed_received(void)
{ ESP_LOGD(TAG, "on_prev_passed_received() sem handler"); }

__attribute__((weak))
void on_spd_received(float speed, uint32_t eta_ms)
{ ESP_LOGD(TAG, "on_spd_received(%.1f,%lu) sem handler",
           speed, (unsigned long)eta_ms); }

__attribute__((weak))
void on_master_claim_received(int from_id)
{ ESP_LOGD(TAG, "on_master_claim_received(%d) sem handler", from_id); }