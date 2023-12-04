#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <esp8266_peri.h>
#include <buffer.h>
#include <ArtNet.h>
#include <WiFiUdp.h>

// Create this file and define the WIFI_SSID and WIFI_PASSWORD
#include <wifi_network.h>

#define STATUS_LED_PIN GPIO_ID_PIN(2)

// Use something like a SN74AHC138 to demultiplex these bits into 4 CS pins
#define CS_DEMULT_PIN_BIT0 GPIO_ID_PIN(4)
#define CS_DEMULT_PIN_BIT1 GPIO_ID_PIN(5)

// I have choosen this value because I think the chance of appearing inside the values is lesser than 0 or 255, or maybe 127/128 value too.
// Less the occurrence of this byte, better will perform this system. However the difference is not so big, as seen in the SPI notes.
#define DATA_CTRL_STUFF_BYTE 0xCA
#define DATA_CTRL_TRANSMISSION_BEGIN 0xF0
#define DATA_CTRL_RECEIVE_NEXT_FRAME 0xF1
#define DATA_CTRL_1BYTE_DATA 0xE0
#define DATA_CTRL_2BYTE_DATA 0xE1
#define DATA_CTRL_1BYTE_ZERO_DATA 0xD0
#define DATA_CTRL_2BYTE_ZERO_DATA 0xD1

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

SPISettings ATTinySPISettings(24 * 100 * 1000, MSBFIRST, SPI_MODE1);

using namespace art_net;

ArtNet MyArtNet;
WiFiUDP UDP;
uint8_t udpBuffer[1024 * 3];

uint8_t dataBuffer[4][2][513];
uint16_t dataSize[4][2];

uint8_t *sendDataPtr[4];
uint8_t *bufferDataPtr[4];

#define LAST_SEND_INDEX_SEND_NEXT_FRAME 0xF000
uint16_t lastSendIndex[4];

uint8_t receiveDataBuffer[2048];
DataBuffer ReceiveDataBuffer(receiveDataBuffer, 2048);

long lastDataReceived = 0;

void onDmxDataSend(uint8_t universe, uint8_t ctrlByte, uint8_t *data, const uint16_t size) {
  if (size <= 512) {
    dataSize[universe][1] = size + 1;
    bufferDataPtr[universe][0] = ctrlByte;
    //memcpy(&bufferDataPtr[universe][1], data, size);

    if (micros() - lastDataReceived > 200000) {
      lastDataReceived = micros();
    }
  }
}

void sendAtrNetPacket(uint32_t dstIP, uint16_t dstPort, uint8_t *data, uint32_t size) {
  UDP.beginPacket(dstIP, dstPort);
  UDP.write(data, size);
  UDP.endPacket();
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(CS_DEMULT_PIN_BIT0, OUTPUT);
  pinMode(CS_DEMULT_PIN_BIT1, OUTPUT);

  Serial.begin(250000, SERIAL_8N1);

  SPI.begin();
  SPI.setHwCs(0);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }

  UDP.begin(0x1936);

  MyArtNet.ip = WiFi.localIP().v4();
  WiFi.macAddress(MyArtNet.mac);
  MyArtNet.setDmxDataCallback(onDmxDataSend);
  MyArtNet.setSendPacketCallback(sendAtrNetPacket);

  sendDataPtr[0] = dataBuffer[0][0];
  sendDataPtr[1] = dataBuffer[1][0];
  sendDataPtr[2] = dataBuffer[2][0];
  sendDataPtr[3] = dataBuffer[3][0];

  bufferDataPtr[0] = dataBuffer[0][1];
  bufferDataPtr[1] = dataBuffer[1][1];
  bufferDataPtr[2] = dataBuffer[2][1];
  bufferDataPtr[3] = dataBuffer[3][1];

  memset(dataSize, 0, sizeof(dataSize));

  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  if (UDP.parsePacket()) {
    size_t read = UDP.read(udpBuffer, sizeof(udpBuffer));
    MyArtNet.onPacketReceived(UDP.remoteIP().v4(), UDP.remotePort(), udpBuffer, read);
  }

  uint8_t moreData = 0;

  if (micros() - lastDataReceived < 100000) {
    GPOS = GPOS | (1 << STATUS_LED_PIN);
  } else {
    GPOC = GPOC | (1 << STATUS_LED_PIN);
  }

    // for (uint8_t universe = 0; universe < 4; universe++) {
    for (uint8_t universe = 0; universe < 1; universe++) {
      // Swap buffers if transmission ended
      if (lastSendIndex[universe] == LAST_SEND_INDEX_SEND_NEXT_FRAME) {
        lastSendIndex[universe] = 0;

        if (dataSize[universe][1] >= 192) {
          dataSize[universe][0] = dataSize[universe][1];
          dataSize[universe][1] = 0;

          uint8_t *aux = sendDataPtr[universe];
          sendDataPtr[universe] = bufferDataPtr[universe];
          bufferDataPtr[universe] = aux;
        }
      }

      // transmit buffer
      // select chip

      // Bit 0
      if (universe & 1) {
        GPOS = GPOS | (1 << CS_DEMULT_PIN_BIT0);
      } else {
        GPOC = GPOC | (1 << CS_DEMULT_PIN_BIT0);
      }

      // Bit 1
      if (universe & 2) {
        GPOS = GPOS | (1 << CS_DEMULT_PIN_BIT1);
      } else {
        GPOC = GPOC | (1 << CS_DEMULT_PIN_BIT1);
      }

      SPI.beginTransaction(ATTinySPISettings);

      uint16_t sendIndex = lastSendIndex[universe];

      if (sendIndex < dataSize[universe][0]) {
        // Send start of transmission if sending first channel    
        if (sendIndex == 0) {
          ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_STUFF_BYTE));
          ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_TRANSMISSION_BEGIN));
        }

        // If data is stuff bit
        if (sendDataPtr[universe][sendIndex] == DATA_CTRL_STUFF_BYTE) {
          ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_STUFF_BYTE));

          // Performance enhancement, if the next byte is the stuff too, we send that there are 2 data bytes
          // This can reduce the maximum amount of transferred data from 1024 bytes to 768 bytes
          if (sendIndex + 1 < dataSize[universe][0] && sendDataPtr[universe][sendIndex + 1] == DATA_CTRL_STUFF_BYTE) {
            ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_2BYTE_DATA));
            lastSendIndex[universe]++;
          } else {
            ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_1BYTE_DATA));
          }
        } else if (sendDataPtr[universe][sendIndex] == 0) {
          // Zero means no data, not a zero data, so we should send the zero inside the CTRL escaping
          ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_STUFF_BYTE));

          // Performance enhancement, if the next byte is the stuff too, we send that there are 2 data bytes
          // This can reduce the maximum amount of transferred data from 1024 bytes to 768 bytes
          if (sendIndex + 1 < dataSize[universe][0] && sendDataPtr[universe][sendIndex + 1] == 0) {
            ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_2BYTE_ZERO_DATA));
            lastSendIndex[universe]++;
          } else {
            ReceiveDataBuffer.enqueueIfValid(SPI.transfer(DATA_CTRL_1BYTE_ZERO_DATA));
          }
        } else {
          ReceiveDataBuffer.enqueueIfValid(SPI.transfer(sendDataPtr[universe][sendIndex]));
        }

        lastSendIndex[universe]++;
      } else {
        ReceiveDataBuffer.enqueueIfValid(SPI.transfer(0));
      }

      while (ReceiveDataBuffer.isEmpty() == 0) {
        if (ReceiveDataBuffer.get() == DATA_CTRL_STUFF_BYTE && ReceiveDataBuffer.used() <= 1) {
          ReceiveDataBuffer.enqueueIfValid(SPI.transfer(0));
          continue;
        }

        uint8_t data = ReceiveDataBuffer.dequeue();

        Serial.print(data);
        Serial.print('\n');

        if (data == DATA_CTRL_STUFF_BYTE) {
          uint8_t ctrlValue = ReceiveDataBuffer.dequeue();

          Serial.print(ctrlValue);
          Serial.print('\n');

          if (ctrlValue == DATA_CTRL_RECEIVE_NEXT_FRAME) {
            lastSendIndex[universe] = LAST_SEND_INDEX_SEND_NEXT_FRAME;
          }
        }
      }

      SPI.endTransaction();
    }
}
