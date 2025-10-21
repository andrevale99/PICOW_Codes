/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"
#include "pico/unique_id.h"
#include "hardware/irq.h"

#include "Ble.h"
#include "Mqtt.h"

void VtaskBLE(void *pvArgs)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskMqtt(void *pvArgs)
{
    MQTT_CLIENT_DATA_T state = *(MQTT_CLIENT_DATA_T *)pvArgs;
    while (1)
    {
        if (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst))
        {
            cyw43_arch_poll();
            // cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//======================================
//  MAIN
//======================================
int main()
{
    stdio_init_all();

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init())
    {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    BLE_Init();

    //======================================
    //  MQTT SETUP
    //======================================
    static MQTT_CLIENT_DATA_T state;

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

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        panic("Failed to connect");
    }
    printf("\nConnected to Wifi\n");

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

    //======================================
    //  TASKS
    //======================================
    xTaskCreate(VtaskBLE, "BLE Task", 128, NULL, 1, NULL);
    xTaskCreate(vTaskMqtt, "MQTT Task", 2048, (void *)&state, 1, NULL);

    vTaskStartScheduler();

    /*
        // btstack_run_loop_execute is only required when using the 'polling' method (e.g. using pico_cyw43_arch_poll library).
        // This example uses the 'threadsafe background` method, where BT work is handled in a low priority IRQ, so it
        // is fine to call bt_stack_run_loop_execute() but equally you can continue executing user code.

    #if 0 // this is only necessary when using polling (which we aren't, but we're showing it is still safe to call in this case)
        btstack_run_loop_execute();
    #else
        // this core is free to do it's own stuff except when using 'polling' method (in which case you should use
        // btstacK_run_loop_ methods to add work to the run loop.

        // this is a forever loop in place of where user code would go.
        while (true)
        {
            sleep_ms(1000);
        }
    #endif
    */
    return 0;
}
