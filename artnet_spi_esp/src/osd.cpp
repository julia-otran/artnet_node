#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <EEPROM.h>
#include "osd.h"
#include <PCF8575.h>

hd44780_I2Cexp lcd(0x27);

PCF8575 keyboard(0x20);

static Settings current_settings;

byte settings_mode_on;
byte selected_setting;
byte current_position;
byte artnet_port_setting;
uint8_t next_setting = 0;
uint8_t prev_setting = 0;
uint8_t dotCount;
uint32_t wifiStatPrintMillis;
uint32_t lastKeyboardPress;

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


uint16_t osd_state;
bool keyboardInit;

Settings* osd_get_settings() {
    return &current_settings;
}

float get_fps_estimate() {
  uint32_t acc_micros = 92 + 12;
  acc_micros += (osd_get_channel_quantity() + 1) * 4 * 11;
  double fps = (1000.0 * 1000.0) / (double) acc_micros;
  return (float) fps;
}


// void loadEEPROM() {
//   byte *settings_raw = (byte*) &current_settings;

//   for (uint16_t i = 0; i < sizeof(current_settings); i++) {
//     settings_raw[i] = EEPROM.read(i);
//   }
// }

// void saveEEPROM() {
//   byte *settings_raw = (byte*) &current_settings;

//   for (uint16_t i = 0; i < sizeof(current_settings); i++) {
//     EEPROM.write(i, settings_raw[i]);
//   }
// }

void osd_init() {
  Wire.begin();
  Wire.setClock(100000);

  for (byte pin = 0; pin < 16; pin++) {
    keyboard.pinMode(pin, INPUT_PULLUP);
  }

  keyboardInit = keyboard.begin();

  lcd.begin(20, 4);
  lcd.createChar(0, ARROW_CHAR);
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

  if (millis() - wifiStatPrintMillis > 250) {
    if ((osd_state >> 8) != (status + 1)) {
      lcd.clear();
      osd_state = (status + 1) << 8;
    }

    if (status == WL_WRONG_PASSWORD) {
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

uint16_t key_press_map;

uint8_t is_key_set(uint8_t key) {
  bool last_state = (key_press_map >> key) & 1;

  if (keyboard.digitalRead(key) == 0) {
    if (last_state == 1) {
      key_press_map = key_press_map & ~((uint16_t)(1 << key));
      return 1;
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

unsigned long last_keyboard_check;

void osd_check_keyboard() {
  if (!keyboardInit) {
    return;
  }  

  if (millis() - last_keyboard_check < 50) {
    return;
  }

  last_keyboard_check = millis();

  if (settings_mode_on == 0 && key_right_set()) {
    settings_mode_on = 1;
    selected_setting = 0;
    current_position = 1;
    artnet_port_setting = 0;
  }
}

uint8_t osd_settings_routine() {
  if (!keyboardInit) {
    return 0;
  }

  if (settings_mode_on == 2 && key_right_set()) {
      settings_mode_on = 1;
      current_position = 1;
      selected_setting = next_setting;
  }

  if (settings_mode_on == 2 && key_left_set()) {
      settings_mode_on = 1;
      current_position = 1;

      if (prev_setting != 255) {
        selected_setting = prev_setting;
      } else {
        settings_mode_on = 0;
        return 2;
      }
  }  

  if (settings_mode_on == 2 && key_up_set() && current_position > 1) {
      settings_mode_on = 1;
      current_position--;
  }

  if (settings_mode_on == 2 && key_down_set()) {
      settings_mode_on = 1;
      current_position++;
  }

  if (settings_mode_on == 1) {
    osd_state = OSD_STATE_SETTINGS;
    settings_mode_on = 2;

    lcd.clear();
    
    
    if (selected_setting == 0) {
      prev_setting = 255;

      lcd.setCursor(0, 0);
      lcd.print("Settings menu:");

      lcd.setCursor(0, 2);

      if (current_position <= 3) {    
        lcd.print("... MORE ...");
      } else {
        lcd.print("--- END ---");
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

      if (current_position == 3) {
        lcd.print("> IP");
        next_setting = 9;
      }

      lcd.setCursor(0, 3);
      lcd.print("< Save | "); lcd.write(0); lcd.print(" Sw OP");
    }

    if (selected_setting == 1 && artnet_port_setting == 0) {
      prev_setting = 0;
      lcd.setCursor(0, 0);
      lcd.print("ArtNet Adressing:");

      lcd.setCursor(0, 2);

      if (current_position <= 3) {    
        lcd.print("... MORE ...");
      } else {
        lcd.print("--- END ---");
        current_position = 4;
      }

      lcd.setCursor(0, 1);

      if (current_position == 1) {
        lcd.print("> Port 1 SRC");
      }

      if (current_position == 2) {
        lcd.print("> Port 2 SRC");
      }

      if (current_position == 3) {
        lcd.print("> Port 3 SRC");
      }

      if (current_position == 4) {
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

      if (current_position <= 2) {    
        lcd.print("... MORE ...");
      } else {
        current_position = 3;
        lcd.print("--- END ---");
      }

      lcd.setCursor(0, 1);

      if (current_position == 1) {
        lcd.print("Net:    ");
        lcd.print(current_settings.address_mapping[artnet_port_setting - 1].net);
      }

      if (current_position == 2) {
        lcd.print("Subnet: ");
        lcd.print(current_settings.address_mapping[artnet_port_setting - 1].subuni >> 4);
      }

      if (current_position == 3) {
        lcd.print("Univer: ");
        lcd.print(current_settings.address_mapping[artnet_port_setting - 1].subuni & 0xF);
      }

      lcd.setCursor(18, 1);
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

      lcd.setCursor(18, 1);
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
      prev_setting = 5;
      next_setting = 6;

      lcd.setCursor(0, 0);
      lcd.print("Select WiFi SSID:");

      lcd.setCursor(0, 2);

      if (current_position <= 1) {    
        lcd.print("... NEXT ...");
      } else {
        current_position = 2;
        lcd.print("--- END ---");
      }

      lcd.setCursor(0, 3);
      lcd.print("< SSID | "); lcd.write(0); lcd.print(" Sw OP");
    }

    if (
      (selected_setting == 5 && (current_settings.wifi_mode & 1)) ||
      (selected_setting == 6 && (current_settings.wifi_mode & 1)) ||
      (selected_setting == 7)
     ) {
      prev_setting = 3;
      next_setting = 7;

      lcd.setCursor(0, 0);
      lcd.print("Type WiFi SSID:");

      lcd.setCursor(0, 2);
      lcd.write(0);
      lcd.print(" Char Pos | O Del");

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
  }

  return settings_mode_on > 0;
}

uint16_t osd_get_channel_quantity() {
  return 192 + (((uint16_t)current_settings.output_channels) * 2);
}

