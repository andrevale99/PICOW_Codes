#ifndef WIFI_H
#define WIFI_H

#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#define WIFI_SUCCESS 0
#define WIFI_ARCH_INIT_ERROR -1
#define WIFI_CONNECTION_FAILED -2

int8_t Wifi_station_init(const char *ssid, const char *password);

#endif