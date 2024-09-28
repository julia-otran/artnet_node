#include <Arduino.h>
#include <ESP8266WiFi.h>

typedef struct {
  char wifi_ssid[200];
  char wifi_pswd[200];
  byte wifi_mode;
  byte static_ip[4];
  byte artnet_net;
  byte artnet_subnet;
  byte port_mapping[4];
  byte output_channels;
} Settings;

Settings* osd_get_settings();
uint16_t osd_get_channel_quantity();
uint8_t osd_settings_routine();
void osd_check_keyboard();
void osd_print_fps(float fps);
void osd_print_wifi_stat(wl_status_t status);
void osd_print_default();
void osd_init();
