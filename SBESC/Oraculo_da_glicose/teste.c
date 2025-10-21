#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/ip_addr.h"

#include "Ble.h"   // Sua biblioteca BLE

#define WIFI_SSID "brisa-2532295" // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD   "zlgy1ssc"
#define MQTT_BROKER     "test.mosquitto.org"
// #define MQTT_PORT       1883

static mqtt_client_t *mqtt_client;
static ip_addr_t mqtt_ip;
static bool mqtt_connected = false;
static bool mqtt_published_once = false;

// ============================ MQTT CALLBACKS ============================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT conectado!\n");
        mqtt_connected = true;

        if (!mqtt_published_once) {
            mqtt_publish(client, "pico/status", "Device online", strlen("Device online"),
                         1, 0, NULL, NULL);
            mqtt_published_once = true;
            printf("Mensagem inicial publicada!\n");
        }
    } else {
        printf("Falha na conexão MQTT: %d\n", status);
        mqtt_connected = false;
    }
}

static void mqtt_connect_broker(void) {
    static struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_ble_mqtt";
    ci.keep_alive = 60;

    mqtt_client = mqtt_client_new();
    if (!mqtt_client) {
        printf("Erro ao criar cliente MQTT\n");
        return;
    }

    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(mqtt_client, &mqtt_ip, MQTT_PORT, mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("mqtt_client_connect retornou erro %d\n", err);
    }
}

// ============================ WIFI + MQTT INIT ============================

static void wifi_mqtt_init(void) {
    printf("Inicializando Wi-Fi...\n");
    if (cyw43_arch_init()) {
        printf("Falha ao inicializar cyw43\n");
        return;
    }
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Falha ao conectar Wi-Fi\n");
        return;
    }
    printf("Wi-Fi conectado!\n");

    printf("Resolvendo broker: %s\n", MQTT_BROKER);
    if (dns_gethostbyname(MQTT_BROKER, &mqtt_ip, NULL, NULL) == ERR_OK) {
        mqtt_connect_broker();
    } else {
        printf("Erro ao resolver broker.\n");
    }
}

// ============================ MAIN LOOP ============================

int main() {
    stdio_init_all();
    printf("\n==== Pico W BLE + MQTT ====\n");

    // Inicializa Wi-Fi e MQTT (publica uma vez)
    wifi_mqtt_init();

    // Inicializa BLE (usa ble.h)
    BLE_Init();

    while (true) {
        // BLE e BTstack rodam seus eventos no loop principal
        btstack_run_loop_execute();

        // (Opcional) Pode publicar manualmente dados BLE aqui:
        /*
        if (mqtt_connected && novo_dado_ble) {
            char msg[32];
            sprintf(msg, "Valor: %u", valor_ble);
            mqtt_publish(mqtt_client, "pico/ble", msg, strlen(msg), 1, 0, NULL, NULL);
        }
        */
    }
}
