#ifndef MQTT_H
#define MQTT_H

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"
#include "pico/unique_id.h"

#include "pico/unique_id.h"
#include "hardware/irq.h"

#define WIFI_SSID "brisa-2532295" // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "zlgy1ssc"
#define MQTT_SERVER "127.0.0.1"

// qos passed to mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0
#define MQTT_TOPIC_LEN 100
#define MQTT_DEVICE_NAME "picoSBESC"

// topic used for last will and testament
#define MQTT_WILL_TOPIC "/ORACULO"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

// keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

typedef struct
{
    mqtt_client_t *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

/// @brief  Callback for publish requests
/// @param arg argumentos
/// @param err ?
static void pub_request_cb(__unused void *arg, err_t err)
{
    if (err != 0)
        printf("pub_request_cb failed %d", err);
}

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name)
{
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

static void publish_temperature(MQTT_CLIENT_DATA_T *state)
{
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = 256;
    if (temperature != old_temperature)
    {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);

        printf("Publishing %s to %s\n", temp_str, temperature_key);

        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str),
                     MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

/// @brief Funcao de callback para requisicoes de inscricao
/// @param arg Estrutura MQTT
/// @param err  ?
static void sub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (err != 0)
        panic("subscribe request failed %d", err);

    state->subscribe_count++;
}

/// @brief Funcao de callback para requisicoes de desinscricao
/// @param arg Estrutura MQTT
/// @param err  ?
static void unsub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (err != 0)
    {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client)
    {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

/// @brief Inscreve ou desinscreve dos topicos
/// @param state Estrutura MQTT
/// @param sub   true para inscrever, false para desinscrever
static void sub_unsub_topics(MQTT_CLIENT_DATA_T *state, bool sub)
{
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/glicose"),
                   MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

/// @brief Funcao que retorna o dado que foi Transmitido
/// do BROKER -> MQTT CLIENT (pico)
/// @param arg Estrutura MQTT
/// @param data Dados que chegaram
/// @param len Tamanho dos dados
/// @param flags ?
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/print") == 0)
    {
        printf("%.*s\n", len, data);
    }
    else if (strcmp(basic_topic, "/ping") == 0)
    {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
    else if (strcmp(basic_topic, "/exit") == 0)
    {
        state->stop_client = true;      // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
}

/// @brief ?
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

/// @brief Funcao que sera chamada periodicamente para publicacao
/// @param context 
/// @param worker 
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)worker->user_data;
    publish_temperature(state);
    async_context_add_at_time_worker_in_ms(context, worker, 10 * 1000);
}

static async_at_time_worker_t temperature_worker = {.do_work = temperature_worker_fn};

/// @brief Funcao de callback de conexao MQTT
/// @param client Cliente (pico w)
/// @param arg Estrutura MQTT
/// @param status Situacao
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic)
        {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1,
                         MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    }
    else if (status == MQTT_CONNECT_DISCONNECTED)
    {
        if (!state->connect_done)
            panic("Failed to connect to mqtt server");
    }
    else
        panic("Unexpected status");
}

/// @brief  Inicia o cliente MQTT
/// @param state Estrutura MQTT
void start_client(MQTT_CLIENT_DATA_T *state)
{
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst)
    {
        panic("MQTT client instance creation error");
    }
    printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port,
                            mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK)
    {
        printf("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb,
                            mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back with a DNS result
void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (ipaddr)
    {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    }
    else
    {
        printf("dns request failed");
    }
}

#endif