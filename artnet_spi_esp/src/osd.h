#include <Arduino.h>
#include <ESP8266WiFi.h>

typedef struct {
  byte net;
  byte subuni;
} ArtNetAddress;

typedef struct {
  char wifi_ssid[200];
  char wifi_pswd[200];
  byte wifi_mode;
  byte static_ip[4];
  ArtNetAddress address_mapping[4];
  byte output_channels;
} Settings;

Settings* osd_get_settings();
uint16_t osd_get_channel_quantity();
uint8_t osd_settings_routine();
void osd_print_fps(float fps);
void osd_print_wifi_stat(wl_status_t status);
void osd_print_default();
void osd_init();
