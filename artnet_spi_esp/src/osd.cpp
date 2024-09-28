#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <EEPROM.h>
#include "osd.h"
#include <PCF8575.h>

#define LCD_COLUMN_COUNT 20

char KEYBOARD_SYMBOLS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890-=+{}[]()<>*&%$#@!\"'\\/;:.,?|Çç ";

hd44780_I2Cexp lcd(0x27);

PCF8575 keyboard(0x20);

static Settings current_settings;
static Settings live_settings;

byte settings_mode_on;
byte selected_setting;
byte current_position;
byte artnet_port_setting;
uint8_t next_setting = 0;
uint8_t prev_setting = 0;
uint8_t dotCount;
uint32_t wifiStatPrintMillis;
bool lastEncoderClock;
byte currentSymbolIndex;

#define OSD_STATE_FPS 1
#define OSD_STATE_SETTINGS 2
#define OSD_STATE_WIFI ((uint8_t)0xF0)

byte ARROW_CHAR[] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100
};

byte ENTER_CHAR[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b01001,
	0b11111,
	0b01000,
	0b00000,
	0b00000
};

byte CHECK_CHAR[8] = {
	0b00000,
	0b00001,
	0b00011,
	0b10110,
	0b11100,
	0b01000,
	0b00000,
	0b00000
};

uint16_t osd_state;
bool keyboardInit;

uint16_t key_press_map;

unsigned long lastKeyPress;

uint8_t is_key_set(uint8_t key) {
  bool last_state = (key_press_map >> key) & 1;

  if (keyboard.digitalRead(key) == 0) {
    if (last_state == 1) {
      if (millis() - lastKeyPress > 500) {
        key_press_map = key_press_map & ~((uint16_t)(1 << key));
        lastKeyPress = millis();
        return 1;
      }
    }
  } else {
    key_press_map |= 1 << key;
  }
  
  return 0;
}

uint8_t key_left_set() {
  return is_key_set(0);
}

uint8_t key_right_set() {
  return is_key_set(1);
}

uint8_t key_up_set() {
  return is_key_set(2);
}

uint8_t key_down_set() {
  return is_key_set(3);
}

uint8_t key_enter_set() {
  return is_key_set(6);
}

Settings* osd_get_settings() {
    return &live_settings;
}

float get_fps_estimate() {
  uint32_t acc_micros = 92 + 12;
  acc_micros += (osd_get_channel_quantity() + 1) * 4 * 11;
  double fps = (1000.0 * 1000.0) / (double) acc_micros;
  return (float) fps;
}

void loadEEPROM() {
  byte *settings_raw = (byte*) &current_settings;

  for (uint16_t i = 0; i < sizeof(current_settings); i++) {
    settings_raw[i] = EEPROM.read(i);
  }

  memcpy(&live_settings, &current_settings, sizeof(Settings));
}

void saveEEPROM() {
  byte *settings_raw = (byte*) &current_settings;

  for (uint16_t i = 0; i < sizeof(current_settings); i++) {
    EEPROM.write(i, settings_raw[i]);
  }
}

void loadWiFiSettings() {
  if (WiFi.status() != WL_IDLE_STATUS) {
    WiFi.disconnect();
  }

  if (current_settings.static_ip[0] == 0 || current_settings.static_ip[3] == 0) {
    current_settings.static_ip[0] = 192;
    current_settings.static_ip[0] = 168;
    current_settings.static_ip[0] = 1;
    current_settings.static_ip[0] = 99;
  }

  if (current_settings.wifi_mode & 1) {
    IPAddress address = IPAddress(
      current_settings.static_ip[0], 
      current_settings.static_ip[1], 
      current_settings.static_ip[2], 
      current_settings.static_ip[3]
    );

    WiFi.softAPConfig(address, IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0));

    if (current_settings.wifi_ssid[0] == 0) {
      current_settings.wifi_ssid[0] = 'J';
      current_settings.wifi_ssid[1] = 'S';
      current_settings.wifi_ssid[2] = 'L';
      current_settings.wifi_ssid[3] = 'C';
      current_settings.wifi_ssid[4] = 0;
    }

    if (current_settings.wifi_pswd[0] == 0) {
      current_settings.wifi_ssid[0] = 'J';
      current_settings.wifi_ssid[1] = 'S';
      current_settings.wifi_ssid[2] = 'L';
      current_settings.wifi_ssid[3] = 'C';
      current_settings.wifi_ssid[4] = '-';
      current_settings.wifi_ssid[5] = 'A';
      current_settings.wifi_ssid[6] = 'R';
      current_settings.wifi_ssid[7] = 'T';
      current_settings.wifi_ssid[8] = 'N';
      current_settings.wifi_ssid[9] = 'E';
      current_settings.wifi_ssid[10] = 'T';
      current_settings.wifi_ssid[11] = 0;
    }

    WiFi.softAP(current_settings.wifi_ssid, current_settings.wifi_pswd);
  } else {
    if (current_settings.wifi_ssid[0] == 0 || current_settings.wifi_pswd[0] == 0) {
      if (current_settings.wifi_mode & 2) {
        IPAddress address = IPAddress(
          current_settings.static_ip[0], 
          current_settings.static_ip[1], 
          current_settings.static_ip[2], 
          current_settings.static_ip[3]
        );

        WiFi.config(address, IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0));
      }

      WiFi.begin(current_settings.wifi_ssid, current_settings.wifi_pswd);
    }
  }
}

void osd_init() {
  loadEEPROM();

  Wire.begin();
  Wire.setClock(100000);

  for (byte pin = 0; pin < 13; pin++) {
    keyboard.pinMode(pin, INPUT_PULLUP);
  }

  keyboard.pinMode(13, OUTPUT, LOW);
  keyboard.pinMode(14, OUTPUT, LOW);
  keyboard.pinMode(15, OUTPUT, LOW);

  keyboardInit = keyboard.begin();

  lcd.begin(LCD_COLUMN_COUNT, 4);
  lcd.createChar(0, ARROW_CHAR);
  lcd.createChar(1, ENTER_CHAR);
  lcd.createChar(2, CHECK_CHAR);
  lcd.setContrast(0x0F);
  lcd.clear();
  lcd.print("JSLC");
  lcd.setCursor(0, 1);
  lcd.print("ArtNet to DMX512");

  if (!keyboardInit) {
    lcd.setCursor(0, 3);
    lcd.print("Keyboard Err!");
  }

  lcd.display();

  if (keyboardInit && key_left_set()) {
    memset(&current_settings, 0, sizeof(Settings));
    memset(&live_settings, 0, sizeof(Settings));
    saveEEPROM();

    lcd.setCursor(0, 3);
    lcd.print("Settings cleared!");
  }

  loadWiFiSettings();
}

String large_string_scroll;
unsigned int position;
unsigned long large_string_scroll_last_increment;

void osd_scroll_routine() {
  unsigned int str_length = large_string_scroll.length();

  if (str_length == 0) {
    position = 0;
    return;
  }

  if (millis() - large_string_scroll_last_increment <= 400) {
    return;
  }

  large_string_scroll_last_increment = millis();

  if (str_length <= LCD_COLUMN_COUNT) {
    lcd.setCursor(0, 1);
    lcd.print(large_string_scroll);
    return;
  }

  for (uint8_t i = 0; i < LCD_COLUMN_COUNT; i++) {
    lcd.setCursor(i, 1);

    if (i + position < str_length) { 
      lcd.print(large_string_scroll[i + position]);
    } else if (i + position == str_length) {
      lcd.print(" ");
    } else {
      lcd.print(large_string_scroll[(i + position) - str_length - 1]);
    }
  }

  position++;

  if (position > str_length + 21) {
    position = 0;
  }
}

void osd_scroll_large_string(String s) {
  large_string_scroll = s;
}

void osd_scroll_large_string_stop() {
  large_string_scroll = "";
}

void osd_print_wifi_connecting() {
    lcd.setCursor(0, 0);
    lcd.print("Connecting WiFi");

    lcd.setCursor(0, 1);

    for (uint8_t i = 0; i < 16; i++) {
        if (i < dotCount) {
            lcd.print(".");
        } else {
            lcd.print(" ");
        }
    }

    lcd.setCursor(0, 3);
    lcd.print("> Settings");

    lcd.display();

    dotCount++;

    if (dotCount >= 16) {
        dotCount = 1;
    }
}

/**
         WL_IDLE_STATUS      = 0,
        WL_NO_SSID_AVAIL    = 1,
        WL_SCAN_COMPLETED   = 2,
        WL_CONNECTED        = 3,
        WL_CONNECT_FAILED   = 4,
        WL_CONNECTION_LOST  = 5,
        WL_DISCONNECTED     = 7
 */

void osd_print_wifi_stat(wl_status_t status) {
  if (settings_mode_on > 0) {
    return;
  }

  keyboard.digitalWrite(13, LOW);

  if (millis() - wifiStatPrintMillis > 250) {
    if ((osd_state >> 8) != (status + 1)) {
      lcd.clear();
      osd_state = (status + 1) << 8;
    }

    if (status == WL_IDLE_STATUS) {
        lcd.setCursor(0, 0);
        lcd.print("WiFi FAILED!");
        lcd.setCursor(0, 1);
        lcd.print("Not configured");
        lcd.setCursor(0, 3);
        lcd.print("> Settings");

        lcd.display();
    } else if (status == WL_WRONG_PASSWORD) {
        lcd.setCursor(0, 0);
        lcd.print("WiFi FAILED!");
        lcd.setCursor(0, 1);
        lcd.print("Wrong Password");
        lcd.setCursor(0, 3);
        lcd.print("> Settings");

        lcd.display();
    } else if (status == WL_CONNECT_FAILED) {
        lcd.setCursor(0, 0);
        lcd.print("WiFi FAILED!");
        lcd.setCursor(0, 1);
        lcd.print("Connection Failed");
        lcd.setCursor(0, 3);
        lcd.print("> Settings");

        lcd.display();
    } else {
        osd_print_wifi_connecting();
    }

    wifiStatPrintMillis = millis();   
  }
}

void osd_print_default() {
  if (settings_mode_on > 0) {
    return;
  }

  if (osd_state != OSD_STATE_FPS) {
    keyboard.digitalWrite(13, HIGH);

    lcd.clear();
    osd_state = OSD_STATE_FPS;
    
    lcd.setCursor(0, 0);
    lcd.print("DMX FPS: ");

    lcd.setCursor(0, 1);

    uint32_t ip = WiFi.localIP().v4();

    lcd.print("IP: ");
    lcd.print((ip) & 0xFF);
    lcd.print(".");
    lcd.print((ip >> 8) & 0xFF);
    lcd.print(".");
    lcd.print((ip >> 16) & 0xFF);
    lcd.print(".");
    lcd.print((ip >> 24) & 0xFF);

    lcd.setCursor(0, 3);
    lcd.print("> Settings");
  }
}

void osd_print_fps(float fps) {
  if (settings_mode_on > 0) {
    return;
  }

  osd_print_default();

  lcd.setCursor(9, 0);
  lcd.print(fps);
  lcd.display();
}

bool last_encoder1;

void encoder_read() {
}

int8_t encoder_value() {
  int8_t encoder_last_read = 0;

  bool encoder1 = keyboard.digitalRead(4, true);
  bool encoder2 = keyboard.digitalRead(5);

  if (encoder1 != last_encoder1 && encoder1 == encoder2) {
    encoder_last_read++;
  }

  if (encoder1 != last_encoder1 && encoder1 != encoder2) {
    encoder_last_read--;
  }

  last_encoder1 = encoder1;

  return encoder_last_read;
}

void osd_check_keyboard() {
  if (!keyboardInit) {
    return;
  }  

  if (settings_mode_on == 0 && key_right_set()) {
    settings_mode_on = 1;
    selected_setting = 0;
    current_position = 1;
    artnet_port_setting = 0;
  }
}

void osd_process_on_screen_keyboard(char* data, uint16_t limit) {
  uint16_t currentCharacterPos = current_position - 1;
  uint16_t stringLength = strlen(data);

  if (currentCharacterPos >= limit) {
    currentCharacterPos = limit - 1;
  }

  int16_t printStartPosition = currentCharacterPos - (LCD_COLUMN_COUNT/2);

  if (printStartPosition < 0) {
    printStartPosition = 0;
  }

  if (printStartPosition - (LCD_COLUMN_COUNT - 1) >= stringLength) {
    printStartPosition = stringLength - LCD_COLUMN_COUNT;
  }

  uint16_t cursorPosition = currentCharacterPos - printStartPosition;

  lcd.noCursor();
  lcd.setCursor(0, 1);
  
  for (uint16_t i = 0; i < (LCD_COLUMN_COUNT - 1); i++) {
    uint16_t position = i + printStartPosition;

    if (position < stringLength && data[position]) {
      lcd.print(data[position]);
    } else {
      lcd.print(" ");
    }
  }

  if (printStartPosition + LCD_COLUMN_COUNT >= stringLength) {
    lcd.setCursor(LCD_COLUMN_COUNT, 1);
    lcd.print(">");
  }

  lcd.setCursor(cursorPosition, 1);
  lcd.cursor();

  if (key_enter_set()) {
    for (uint16_t i = currentCharacterPos; i < stringLength - 1; i++) {
      data[i] = data[i+1];
    }

    data[stringLength - 1] = 0;

    if (currentCharacterPos > 0 && data[currentCharacterPos] == 0 && current_position > 1) {
      current_position--;
    } 
  } else {
    uint8_t currentCharIndex = 0;

    if (data[currentCharacterPos] == 0) {
      data[currentCharacterPos] = KEYBOARD_SYMBOLS[0];
      data[currentCharacterPos + 1] = 0;
    }

    for (uint8_t i = 0; i < sizeof(KEYBOARD_SYMBOLS); i++) {
      if (KEYBOARD_SYMBOLS[i] == data[currentCharacterPos]) {
        currentCharIndex = i;
      }
    }

    currentCharIndex += encoder_value();

    if (currentCharIndex > sizeof(KEYBOARD_SYMBOLS) - 1) {
      currentCharIndex = 0;
    }

    data[currentCharacterPos] = KEYBOARD_SYMBOLS[currentCharIndex];
  }
}

void osd_process_change_setting() {
  osd_scroll_routine();

  if (selected_setting == 1) {
    if (artnet_port_setting > 0) {
      int8_t inc = encoder_value();

      if (inc) {
        uint8_t universe = current_settings.port_mapping[artnet_port_setting - 1];
        universe += inc;
        settings_mode_on = 1;
      
        if (universe > 0xF) {
          universe = 0;
        }

        current_settings.port_mapping[artnet_port_setting - 1] = universe;
      }
      
    } else if (current_position == 1) {
      int8_t inc = encoder_value();

      if (inc != 0) {
        current_settings.artnet_net += inc;

        if (current_settings.artnet_net > 0x7F) {
          current_settings.artnet_net = 0;
        }

        settings_mode_on = 1;
      }
    } else if (current_position == 2) {
      int8_t inc = encoder_value();

      if (inc != 0) {
        current_settings.artnet_subnet += inc;

        if (current_settings.artnet_subnet > 0xF) {
          current_settings.artnet_subnet = 0;
        }

        settings_mode_on = 1;
      }
    }
  }

  if (selected_setting == 2) {
    int8_t inc = encoder_value();

    if (inc) {
      current_settings.output_channels += inc;

      if (current_settings.output_channels > 160) {
        current_settings.output_channels = 0;
      }

      settings_mode_on = 1;
    }
  }

  if (selected_setting == 4) {
    bool wifiMode = current_settings.wifi_mode & 1;

    if (encoder_value()) {
      wifiMode = !wifiMode;
      settings_mode_on = 1;
    }

    current_settings.wifi_mode = (current_settings.wifi_mode & 2) | wifiMode;
  }

  if (selected_setting == 6) {
    int n = WiFi.scanComplete();

    if (n < 0) {
      lcd.setCursor(0, 1);
      lcd.print("Searching...");
    } else if (n == 0) {
      lcd.setCursor(0, 1);
      lcd.print("None found!");
    } else if (current_position == 1) {
      lcd.setCursor(0, 1);
      lcd.printf("%03d", n);
      lcd.print(" SSIDs found ");
      lcd.write(0);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(". NEXT .            ");
    }

    if (key_enter_set() && n > 0 && current_position > 1) {
      String selectedSSID = WiFi.SSID(current_position - 2);

      if (selectedSSID.length() > 0 && selectedSSID.length() - 1 < sizeof(current_settings.wifi_ssid)) {
        for (uint8_t i = 0; i < selectedSSID.length(); i++) {
          current_settings.wifi_ssid[i] = selectedSSID.charAt(i);
        }
        current_settings.wifi_ssid[selectedSSID.length()] = 0;

        settings_mode_on = 1;
      }
    }
  }

  if (
    (selected_setting == 5 && (current_settings.wifi_mode & 1)) ||
    (selected_setting == 6 && (current_settings.wifi_mode & 1)) ||
    (selected_setting == 7)
  ) {
    osd_process_on_screen_keyboard(current_settings.wifi_ssid, sizeof(current_settings.wifi_ssid));
  }

  if (selected_setting == 8) {
    osd_process_on_screen_keyboard(current_settings.wifi_pswd, sizeof(current_settings.wifi_pswd));
  }

  if (selected_setting == 10) {
    bool dhcp_disable = current_settings.wifi_mode & 2;

    if (encoder_value()) {
      dhcp_disable = !dhcp_disable;
      settings_mode_on = 1;
    }

    current_settings.wifi_mode = (current_settings.wifi_mode & 1) | (dhcp_disable << 1);
  }

  if (selected_setting == 11 && (current_settings.wifi_mode & 2) == 0) {
    uint8_t ipIndex = current_position - 1;
    int8_t inc = encoder_value();

    if (inc) {
      current_settings.static_ip[ipIndex] += inc;
      settings_mode_on = 1;
    }
  }  
}

uint8_t osd_settings_routine() {
  if (!keyboardInit) {
    return 0;
  }

  if (settings_mode_on == 2 && key_right_set()) {
      if (selected_setting == 1 && next_setting == 1 && current_position > 2) {
        artnet_port_setting = current_position - 2;
      }

      settings_mode_on = 1;
      current_position = 1;
      selected_setting = next_setting;
  }

  if (settings_mode_on == 2 && key_left_set()) {
    osd_scroll_large_string_stop();

    if (prev_setting != 255) {
      if (selected_setting == 1 && artnet_port_setting > 0) {
        artnet_port_setting = 0;
      } else {
        selected_setting = prev_setting;
      }
    } else {
      settings_mode_on = 0;
      memcpy(&current_settings, &live_settings, sizeof(Settings));
      return 0;
    }

    settings_mode_on = 1;
    current_position = 1;
  }

  if (settings_mode_on == 2 && prev_setting == 255 && key_enter_set()) {
    memcpy(&live_settings, &current_settings, sizeof(Settings));
    saveEEPROM();
    loadWiFiSettings();
    return 2;
  }

  if (settings_mode_on == 2 && key_up_set() && current_position > 1) {
      settings_mode_on = 1;
      current_position--;
  }

  if (settings_mode_on == 2 && key_down_set()) {
      settings_mode_on = 1;
      current_position++;
  }

  if (settings_mode_on == 2) {
    osd_process_change_setting();
    encoder_read();
  }

  if (settings_mode_on != 1) {
    return settings_mode_on > 0;
  }

  osd_state = OSD_STATE_SETTINGS;
  keyboard.digitalWrite(13, LOW);
  settings_mode_on = 2;

  lcd.clear();
  lcd.noCursor();
  
  if (selected_setting == 0) {
    prev_setting = 255;

    lcd.setCursor(0, 0);
    lcd.print("Settings menu:");

    lcd.setCursor(0, 2);

    if (current_position <= 3) {    
      lcd.write(1);
      lcd.print(" Save   | . MORE .");
    } else {
      lcd.write(1);
      lcd.print(" Save   | - END -");
      current_position = 4;
    }

    lcd.setCursor(0, 1);

    if (current_position == 1) {
      lcd.print("> ArtNet Addressing");
      next_setting = 1;
    }

    if (current_position == 2) {
      lcd.print("> Output Channels");
      next_setting = 2;
    }

    if (current_position == 3) {
      lcd.print("> WiFi");
      next_setting = 3;
    }

    if (current_position == 4) {
      lcd.print("> IP");
      next_setting = 9;
    }

    lcd.setCursor(0, 3);
    lcd.print("< Cancel | "); lcd.write(0); lcd.print(" Sw OP");
  }

  if (selected_setting == 1 && artnet_port_setting == 0) {
    prev_setting = 0;
    lcd.setCursor(0, 0);
    lcd.print("ArtNet Adressing:");

    lcd.setCursor(0, 2);

    if (current_position <= 5) {    
      lcd.print("... MORE ...");
    } else {
      lcd.print("--- END ---");
      current_position = 6;
    }

    lcd.setCursor(0, 1);

    if (current_position == 1) {
      lcd.print("> Net: ");
      lcd.print(current_settings.artnet_net);
      lcd.setCursor(17, 1);
      lcd.print("+/-");
    }

    if (current_position == 2) {
      lcd.print("> Subnet: ");
      lcd.print(current_settings.artnet_subnet);
      lcd.setCursor(17, 1);
      lcd.print("+/-");
    }

    if (current_position == 3) {
      lcd.print("> Port 1 SRC");
    }

    if (current_position == 4) {
      lcd.print("> Port 2 SRC");
    }

    if (current_position == 5) {
      lcd.print("> Port 3 SRC");
    }

    if (current_position == 6) {
      lcd.print("> Port 4 SRC");
    }

    lcd.setCursor(0, 3);
    lcd.print("< Sett | "); lcd.write(0); lcd.print(" Sw OP");
  }
  
  if (selected_setting == 1 && artnet_port_setting > 0) {
    lcd.setCursor(0, 0);
    lcd.print("Port ");
    lcd.print(artnet_port_setting);
    lcd.print(" ArtNet Addr:");

    lcd.setCursor(0, 2);

    if (current_position <= 1) {    
      current_position = 1;
      lcd.print("--- END ---");
    }

    lcd.setCursor(0, 1);
    lcd.print("Univer: ");
    lcd.print(current_settings.port_mapping[artnet_port_setting - 1]);

    lcd.setCursor(17, 1);
    lcd.print("+/-");

    lcd.setCursor(0, 3);
    lcd.print("< Ports | "); lcd.write(0); lcd.print(" Sw OP");
  }

  if (selected_setting == 2) {
    prev_setting = 0;
    next_setting = 2;

    lcd.setCursor(0, 0);
    lcd.print("Output Channels:");

    lcd.setCursor(0, 1);
    lcd.print("Qty: ");
    lcd.print(osd_get_channel_quantity());

    lcd.setCursor(17, 1);
    lcd.print("+/-");

    lcd.setCursor(0, 2);
    lcd.print("FPS: ");
    lcd.print(get_fps_estimate());

    lcd.setCursor(0, 3);
    lcd.print("< Settings");
  }

  if (selected_setting == 3) {
    prev_setting = 0;

    lcd.setCursor(0, 0);
    lcd.print("WiFi:");

    lcd.setCursor(0, 2);

    if (current_position <= 2) {    
      lcd.print("... MORE ...");
    } else {
      lcd.print("--- END ---");
      current_position = 3;
    }

    lcd.setCursor(0, 1);

    if (current_position == 1) {
      lcd.print("> Mode (AP/Client)");
      next_setting = 4;
    }

    if (current_position == 2) {
      lcd.print("> SSID");
      next_setting = 5;
    }

    if (current_position == 3) {
      lcd.print("> Password");
      next_setting = 8;
    }

    lcd.setCursor(0, 3);
    lcd.print("< Sett| "); lcd.write(0); lcd.print(" Sw OP");
  }

  if (selected_setting == 4) {
    prev_setting = 3;

    lcd.setCursor(0, 0);
    lcd.print("WiFi Mode:");

    lcd.setCursor(0, 1);

    if (current_settings.wifi_mode & 1) {
      lcd.print("> [AP] |  Client ");
    } else {
      lcd.print(">  AP  | [Client]");
    }

    lcd.setCursor(0, 3);
    lcd.print("< Sett | +/- Sw OP");
  }

  if (selected_setting == 5 && (current_settings.wifi_mode & 1) == 0) {
    prev_setting = 3;

    lcd.setCursor(0, 0);
    lcd.print("WiFi SSID:");

    lcd.setCursor(0, 2);

    if (current_position <= 1) {    
      lcd.print("... MORE ...");
    } else {
      current_position = 2;
      lcd.print("--- END ---");
    }

    lcd.setCursor(0, 1);

    if (current_position == 1) {
      lcd.print("> Search for SSIDs");
      next_setting = 6;
    }

    if (current_position == 2) {
      lcd.print("> Enter Manually");
      next_setting = 7;
    }

    lcd.setCursor(0, 3);
    lcd.print("< WiFi | "); lcd.write(0); lcd.print(" Sw OP");
  }

  if (selected_setting == 6 && (current_settings.wifi_mode & 1) == 0) {
    if (prev_setting != 5) {
      if (WiFi.scanComplete() > 0) {
        WiFi.scanDelete();
      }

      WiFi.scanNetworks(true);
    }

    prev_setting = 5;
    next_setting = 6;

    lcd.setCursor(0, 0);
    lcd.print("Select WiFi SSID:");

    lcd.setCursor(0, 2);

    int8_t countNetworks = WiFi.scanComplete();

    if (countNetworks > 0 && current_position <= countNetworks + 1) {    
      lcd.print("NEXT");

      if (current_position > 1) {
        String ssid = WiFi.SSID(current_position - 2);

        lcd.print(" | ");

        if (ssid.equals(current_settings.wifi_ssid)) {
          lcd.write(2);
          lcd.print(" Selected");
        } else {
          lcd.write(1);
          lcd.print(" Select");
        }
        
        lcd.setCursor(0, 1);
        osd_scroll_large_string(ssid);
      }
    } else {
      if (countNetworks > 0) {
        current_position = countNetworks + 2;
      }

      lcd.print("- END - | ");
      lcd.write(1);
      lcd.print(" Select");
    }

    lcd.setCursor(0, 3);
    lcd.print("< SSID | "); lcd.write(0); lcd.print(" Sw OP");
  }

  if (
    (selected_setting == 5 && (current_settings.wifi_mode & 1)) ||
    (selected_setting == 6 && (current_settings.wifi_mode & 1)) ||
    (selected_setting == 7)
    ) {
    if (current_settings.wifi_mode & 1) {
      prev_setting = 3;
    } else {
      prev_setting = 5;
    }

    next_setting = 7;

    lcd.setCursor(0, 0);
    lcd.print("Type WiFi SSID:");

    lcd.setCursor(0, 2);
    lcd.write(0);
    lcd.print(" Char Pos | ");
    lcd.write(1);
    lcd.print(" Del");

    lcd.setCursor(0, 3);
    lcd.print("< Confirm | +/- Char");
  }

  if (selected_setting == 8) {
    lcd.setCursor(0, 0);
    lcd.print("Type WiFi Password:");

    lcd.setCursor(0, 2);
    lcd.write(0);
    lcd.print(" Char Pos | O Del");

    lcd.setCursor(0, 3);
    lcd.print("< Confirm | +/- Char");
  }

  if (selected_setting == 9) {
    prev_setting = 0;
    lcd.setCursor(0, 0);
    lcd.print("IP:");

    lcd.setCursor(0, 1);

    if (current_position == 1) {
      lcd.print("> Mode");
      next_setting = 10;
    }

    if ((current_settings.wifi_mode & 2) == 0) {
      lcd.setCursor(0, 2);
      lcd.print("--- END ---");

      lcd.setCursor(0, 3);
      lcd.print("< IP");
    } else {
      lcd.setCursor(0, 2);
      if (current_position <= 1) {    
        lcd.print("... MORE ...");
      } else {
        current_position = 2;
        lcd.print("--- END ---");
      }

      if (current_position == 2) {
        lcd.print("> Set Addr");
        next_setting = 11;
      }

      lcd.setCursor(0, 3);
      lcd.print("< IP | "); lcd.write(0); lcd.print(" Sw OP");
    }
  }

  if (selected_setting == 10) {
    prev_setting = 9;
    lcd.setCursor(0, 0);
    lcd.print("IP Mode:");

    lcd.setCursor(0, 1);

    if ((current_settings.wifi_mode & 1) == 0) {
      if ((current_settings.wifi_mode & 2) == 0) {
        lcd.print("[DHCP Cli] |  Fixed ");
      } else {
        lcd.print(" DHCP Cli  | [Fixed]");
      }
    } else {
      if ((current_settings.wifi_mode & 2) == 0) {
        lcd.print("[DHCP Svr] |  Fixed ");
      } else {
        lcd.print(" DHCP Svr  | [Fixed]");
      }
    }

    lcd.setCursor(0, 3);
    lcd.print("< IP | +/- Change");
  }

  if (selected_setting == 11 && (current_settings.wifi_mode & 2) == 0) {
    prev_setting = 9;
    lcd.setCursor(0, 0);
    lcd.print("Set IP Addr:");
    
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(current_settings.static_ip[0]);
    lcd.print(".");
    lcd.print(current_settings.static_ip[1]);
    lcd.print(".");
    lcd.print(current_settings.static_ip[2]);
    lcd.print(".");
    lcd.print(current_settings.static_ip[3]);

    lcd.setCursor(0, 2);
    lcd.print("+/- Change Number");

    lcd.setCursor(0, 3);
    lcd.print("< IP | "); lcd.write(0); lcd.print(" Ch Block");

    if (current_position > 4) {
      current_position = 4;
    }

    if (current_position == 1) {
      lcd.setCursor(4, 1);
    }

    if (current_position == 2) {
      lcd.setCursor(8, 1);
    }

    if (current_position == 3) {
      lcd.setCursor(12, 1);
    }

    if (current_position == 4) {
      lcd.setCursor(16, 1);
    }

    lcd.cursor();
  }

  lcd.display();

  return settings_mode_on > 0;
}

uint16_t osd_get_channel_quantity() {
  return 192 + (((uint16_t)current_settings.output_channels) * 2);
}

