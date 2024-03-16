#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <esp8266_peri.h>
#include <buffer.h>
#include <ArtNet.h>
#include <WiFiUdp.h>
#include <uart.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd(0x27);

// Create this file and define the WIFI_SSID and WIFI_PASSWORD
#include <wifi_network.h>

#define DELTA(a,b) ((a > b) ? (a) - (b) : (b) - (a))

// Use something like a SN74AHC138 to demultiplex these bits into 4 CS pins
#define CS_DEMULT_PIN_BIT0 GPIO_ID_PIN(0)
#define CS_DEMULT_PIN_BIT1 GPIO_ID_PIN(15)
#define DMX_BREAK_PIN GPIO_ID_PIN(2)

// Note:
// 1. ATtiny max SCK is 5Mhz, so setting to 4.8Mhz
// 2. I've used MSBFIRST and SPI_MODE0, the default for ATtiny. This can be changed, however must match ATtiny settings.
// 3. We can transfer 2.4Mbit/s. This is sufficient:
//
// Every DMX frame has the max of 512 data bytes.
// Consider we have a stuff, with a max of (768 + 2 start bytes) / frame -> 770 bytes / frame
// 770 bytes * 8 bit/byte = 6160 bits / frame

// We have 4 universes sharing the same SPI network, so
// 6160 bits / frame * 4 = 24640 bits / 4 frames

// So data transfer will took 24640 bits / 2,400,000 bit/sec = 10.27ms / 4 frames

// The DMX512 proto requires approx. 22ms to transmit a frame.
// Our SPI should take a bit more than 10.27ms per 4 frame (consider time took to switch Chip Select and transactions begin / end)
// However, this is less than a half of the time took for writing 1 frame into DMX512 cable.

SPISettings ATTinySPISettings(15 * 100 * 1000, MSBFIRST, SPI_MODE1);

using namespace art_net;

ArtNet MyArtNet;
WiFiUDP UDP;
uint32_t udpBuffer[512];

uint8_t txDataBufferArr[4][2][1024];

uint8_t txInputDataBufferReady[4];
uint8_t *txInputDataBuffers[4];

uint8_t *txOutputDataBuffers[4];

uint16_t dmxChannels;
uint16_t clockWriteCount;
uint16_t currentTxIndex;

uint32_t SPI_BIT_COUNT_MASK;

uint32_t frameCount;
unsigned long lastFrameCheckInterval;

unsigned long breakStartAt;

uint16_t clockWriteCntTotal() {
  uint32_t bitsToSend = (dmxChannels + 1) * 11;
  uint32_t clockSentPerByte = 5;

  return (bitsToSend + (clockSentPerByte - 1)) / clockSentPerByte;
}

void onDmxDataSend(uint8_t universe, uint8_t ctrlByte, uint8_t *data, const uint16_t size) {
  if (universe < 4) {
    uint16_t copyCount = size < dmxChannels ? size : dmxChannels;
    uint8_t *dstStart = txInputDataBuffers[universe];
    uint8_t *dst = &dstStart[1];

    dstStart[0] = ctrlByte;
    memcpy(dst, data, copyCount);

    txInputDataBufferReady[universe] = 1;
  }
}

void sendAtrNetPacket(uint32_t dstIP, uint16_t dstPort, uint8_t *data, uint32_t size) {
  UDP.beginPacket(dstIP, dstPort);
  UDP.write(data, size);
  UDP.endPacket();
}

void setup() {
  dmxChannels = 512;

  pinMode(CS_DEMULT_PIN_BIT0, OUTPUT);
  pinMode(CS_DEMULT_PIN_BIT1, OUTPUT);
  pinMode(DMX_BREAK_PIN, OUTPUT);

  GPOS = 1 << CS_DEMULT_PIN_BIT0;
  GPOS = 1 << CS_DEMULT_PIN_BIT1;
  GPOC = 1 << DMX_BREAK_PIN;

  pinMode(GPIO_ID_PIN(5), OUTPUT);
  pinMode(GPIO_ID_PIN(4), OUTPUT);

  Wire.begin();
  Wire.setClock(100000);

  lcd.begin(16, 2);
  lcd.setContrast(0x0F);
  lcd.clear();
  lcd.print("JSLC");
  lcd.setCursor(0, 1);
  lcd.print("ArtNet to DMX512");
  lcd.display();

  delay(1750);
  optimistic_yield(2000);

  SPI.begin();
  SPI.setHwCs(0);
  SPI.beginTransaction(ATTinySPISettings);

  const uint32_t bits = (1 * 8) - 1;
  const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  SPI_BIT_COUNT_MASK = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));

  lcd.clear();
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  lcd.print(".");
  lcd.display();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t dotCount = 0;

  while (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor(0, 1);
    for (uint8_t i = 0; i < 16; i++) {
      if (i < dotCount) {
        lcd.print(".");
      } else {
        lcd.print(" ");
      }
    }

    lcd.display();
    dotCount++;

    if (dotCount >= 16) {
      dotCount = 1;
    }
    
    optimistic_yield(250);
  }

  UDP.begin(0x1936);

  MyArtNet.ip = WiFi.localIP().v4();
  WiFi.macAddress(MyArtNet.mac);
  MyArtNet.setDmxDataCallback(onDmxDataSend);
  MyArtNet.setSendPacketCallback(sendAtrNetPacket);

  txInputDataBuffers[0] = txDataBufferArr[0][0];
  txInputDataBuffers[1] = txDataBufferArr[1][0];
  txInputDataBuffers[2] = txDataBufferArr[2][0];
  txInputDataBuffers[3] = txDataBufferArr[3][0];

  txOutputDataBuffers[0] = txDataBufferArr[0][1];
  txOutputDataBuffers[1] = txDataBufferArr[1][1];
  txOutputDataBuffers[2] = txDataBufferArr[2][1];
  txOutputDataBuffers[3] = txDataBufferArr[3][1];

  Serial.begin(500000, SERIAL_8N1);

  lastFrameCheckInterval = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DMX FPS: ");

  breakStartAt = 0;
}

void loop() {
  if (currentTxIndex >= (dmxChannels / 2) && UDP.parsePacket()) {
    size_t read = UDP.read((uint8_t*)udpBuffer, sizeof(udpBuffer));
    MyArtNet.onPacketReceived(UDP.remoteIP().v4(), UDP.remotePort(), (uint8_t*)udpBuffer, read);
  }

  if (currentTxIndex >= (dmxChannels + 1) && clockWriteCount >= clockWriteCntTotal()) {
    if (UART_TX_FIFO_SIZE - Serial.availableForWrite() <= 1) {
      Serial.flush();

      GPOS = 1 << DMX_BREAK_PIN;
      breakStartAt = micros();

      for (uint8_t universe = 0; universe < 4; universe++) {
        if (txInputDataBufferReady[universe]) {
          txInputDataBufferReady[universe] = 0;

          uint8_t *txInput = txInputDataBuffers[universe];
          uint8_t *txOut = txOutputDataBuffers[universe];

          txInputDataBuffers[universe] = txOut;
          txOutputDataBuffers[universe] = txInput;
        }
      }

      currentTxIndex = 0;
      clockWriteCount = 0;
      frameCount++;
    } else if (frameCount > 90) {
      unsigned long now = millis();
      unsigned long interval = now - lastFrameCheckInterval;

      unsigned long fps = frameCount * 1000 * 100 / interval;

      lcd.setCursor(9, 0);
      lcd.print(fps / 100.0);
      lcd.display();

      lastFrameCheckInterval = now;
      frameCount = 0;
    }
  }

  // IO
  SPI1U1 = SPI_BIT_COUNT_MASK;

  for (uint8_t countBytes = 0; countBytes < 32; countBytes++) {
    if (currentTxIndex >= (dmxChannels + 1)) {
      break;
    }

    for (uint8_t universe = 0; universe < 4; universe++) {
      uint8_t *txOutputBuffer = txOutputDataBuffers[universe];

      while(SPI1CMD & SPIBUSY) {}

      if (universe == 0) {
        GPOC = 1 << CS_DEMULT_PIN_BIT0;
        GPOS = 1 << CS_DEMULT_PIN_BIT1;
      } else if (universe == 1) {
        GPOS = 1 << CS_DEMULT_PIN_BIT0;
        GPOC = 1 << CS_DEMULT_PIN_BIT1;
      } else {
        GPOS = 1 << CS_DEMULT_PIN_BIT0;
        GPOS = 1 << CS_DEMULT_PIN_BIT1;
      }

      SPI1W0 = txOutputBuffer[currentTxIndex];
      SPI1CMD |= SPIBUSY;

      if (breakStartAt != 0) {
        if (DELTA(breakStartAt, micros()) > 96) {
          breakStartAt = 0;
        } else if (DELTA(breakStartAt, micros()) > 88) {
          GPOC = 1 << DMX_BREAK_PIN;
        }
      }

      if (breakStartAt == 0 && (clockWriteCount + 3 < clockWriteCntTotal()) && Serial.availableForWrite() > 5) {
        Serial.write((uint8_t)0b01010101);
        Serial.write((uint8_t)0b01010101);
        Serial.write((uint8_t)0b01010101);
        clockWriteCount += 3;
      }  
    }

    currentTxIndex++;
  }

  while (Serial.availableForWrite() > 4 && clockWriteCount < clockWriteCntTotal()) {
    Serial.write((uint8_t)0b01010101);
    clockWriteCount++;
  }
}
