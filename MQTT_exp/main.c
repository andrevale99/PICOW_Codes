#include <stdio.h>

#include "pico/stdlib.h"

#include "Wifi.h"
#include "Mqtt.h"

int main()
    {

        MQTT_CLIENT_DATA_T xMqttData;

    stdio_init_all();

    Wifi_station_init("brisa-2532295", "zlgy1ssc");

    Mqtt_init(&xMqttData);

    while (true)
        {

        }
    }
