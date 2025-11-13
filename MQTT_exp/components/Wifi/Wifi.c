#include "Wifi.h"

int8_t Wifi_station_init(const char *ssid, const char *password)
  {
  if (cyw43_arch_init())
    return WIFI_ARCH_INIT_ERROR;

  cyw43_arch_enable_sta_mode();
  if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 30000))
    return WIFI_CONNECTION_FAILED;

  return WIFI_SUCCESS;
  }