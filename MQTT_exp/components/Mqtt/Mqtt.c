/*
Procure por SBESC para procurar
as linhas para revisao
*/

#include "Mqtt.h"

static MQTT_CLIENT_DATA_T state;

//========================================
//  CALLBACKS FUNCTIONS
//========================================

static const char* full_topic(MQTT_CLIENT_DATA_T* state, const char* name)
    {
#if MQTT_UNIQUE_TOPIC
    static char full_topic_[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
    }

/// @brief Callback para qunado realizar a conexao
/// @param[in] client Estrutura mqtt_client_t 
/// @param[in] arg Argumentos 
/// @param[in] status Status de conexao do MQTT
static void mqtt_connection_cb(mqtt_client_t* client, void* arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;

    switch (status)
        {
        case MQTT_CONNECT_ACCEPTED:
        {
        state->connect_done = true;

        printf("[MQTT] CONEXAO ACEITA\n");

        // sbesc
        // //Necessidade de se inscrever ?
        // sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic)
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic,
                "1", 1, MQTT_WILL_QOS, true, NULL, state);


        // sbesc
        // // Publish temperature every 10 sec if it's changed
        // temperature_worker.user_data = state;
        // async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
        }
        break;

        case MQTT_CONNECT_DISCONNECTED:
        {
        if (!state->connect_done)
            printf("[MQTT] Falha na conexao MQTT\n");

        else
            printf("[MQTT] Falha na conexao MQTT desconhecida\n");
        }
        break;

        default:
            break;
        }


    }

static void mqtt_incoming_publish_cb(void* arg, const char* topic, u32_t tot_len)
    {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
    }

//========================================
//  STATIC FUNCTIONS
//========================================

/// @brief  Inicia o cliente MQTT
/// @param state Estrutura MQTT
static void start_client(MQTT_CLIENT_DATA_T* state)
    {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    // printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst)
        panic("MQTT client instance creation error");

    // printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    // printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port,
        mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK)
        printf("MQTT broker connection error");

    printf("[MQTT] CONECTADO\n");
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb,
        NULL, state);
    cyw43_arch_lwip_end();
    }

//========================================
//  PUBLIC FUNCTIONS
//========================================

int8_t Mqtt_init(MQTT_CLIENT_DATA_T* state_)
    {
    //  static MQTT_CLIENT_DATA_T state;
    state = *state_;
    // Use board unique id
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for (int i = 0; i < sizeof(unique_id_buf) - 1; i++)
        {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
        }

    // Generate a unique name, e.g. pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
        client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif

#endif

    // We are not in a callback so locking is needed when calling lwip
     // Make a DNS request for the MQTT server IP address
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK)
        {
        // We have the address, just start the client
        start_client(&state);
        }
    else if (err != ERR_INPROGRESS)
        { // ERR_INPROGRESS means expect a callback
        panic("dns request failed");
        }

    }

// Call back with a DNS result
void dns_found(const char* hostname, const ip_addr_t* ipaddr, void* arg)
    {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr)
        {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
        }
    else
        printf("dns request failed");
    }