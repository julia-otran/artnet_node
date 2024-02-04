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

uint8_t txDataBufferArr[4][2][2048];
DataBuffer *txInputDataBuffers[4];
DataBuffer *txOutputDataBuffers[4];

#define LAST_SEND_INDEX_SEND_NEXT_FRAME 0xF000
uint16_t lastSendIndex[4];

uint8_t receiveDataBufferArr[4][256];
DataBuffer* receiveDataBuffer[4];

long lastDataReceived = 0;
uint8_t currentSPITransferUniverse;

uint32_t SPI_BIT_COUNT_MASK;

void onDmxDataSend(uint8_t universe, uint8_t ctrlByte, uint8_t *data, const uint16_t size) {
  if (size <= 512) {
    txInputDataBuffers[universe]->clear();
    txInputDataBuffers[universe]->enqueue(DATA_CTRL_STUFF_BYTE);
    txInputDataBuffers[universe]->enqueue(DATA_CTRL_TRANSMISSION_BEGIN);

    uint8_t nextVal = ctrlByte;
    uint16_t currentPos = -1;

    while (currentPos < size) {
      if (nextVal == 0) {
        txInputDataBuffers[universe]->enqueue(DATA_CTRL_STUFF_BYTE);

        if (currentPos + 1 < size && data[currentPos + 1] == 0) {  
          txInputDataBuffers[universe]->enqueue(DATA_CTRL_2BYTE_ZERO_DATA);
          currentPos += 2;
        } else {
          txInputDataBuffers[universe]->enqueue(DATA_CTRL_1BYTE_ZERO_DATA);
          currentPos += 1;
        }
      } else if (nextVal == DATA_CTRL_STUFF_BYTE) {
        txInputDataBuffers[universe]->enqueue(DATA_CTRL_STUFF_BYTE);

        if (currentPos + 1 < size && data[currentPos + 1] == 0) {  
          txInputDataBuffers[universe]->enqueue(DATA_CTRL_2BYTE_DATA);
          currentPos += 2;
        } else {
          txInputDataBuffers[universe]->enqueue(DATA_CTRL_1BYTE_DATA);
          currentPos += 1;
        }
      } else {
        txInputDataBuffers[universe]->enqueue(nextVal);
        currentPos += 1;
      }

      nextVal = data[currentPos];
    }

    while (currentPos < 191) {
      txInputDataBuffers[universe]->enqueue(DATA_CTRL_STUFF_BYTE);
      txInputDataBuffers[universe]->enqueue(DATA_CTRL_2BYTE_ZERO_DATA);
      currentPos += 2;
    }

    if (currentPos < 192) {
      txInputDataBuffers[universe]->enqueue(DATA_CTRL_STUFF_BYTE);
      txInputDataBuffers[universe]->enqueue(DATA_CTRL_1BYTE_ZERO_DATA);
      currentPos += 1;
    }

    /*
    if (micros() - lastDataReceived > 200000) {
      lastDataReceived = micros();
    }
    */
  }
}

void sendAtrNetPacket(uint32_t dstIP, uint16_t dstPort, uint8_t *data, uint32_t size) {
  UDP.beginPacket(dstIP, dstPort);
  UDP.write(data, size);
  UDP.endPacket();
}

void setup() {
  GPOS = GPOS | (1 << CS_DEMULT_PIN_BIT0);
  GPOS = GPOS | (1 << CS_DEMULT_PIN_BIT1);

  pinMode(CS_DEMULT_PIN_BIT0, OUTPUT);
  pinMode(CS_DEMULT_PIN_BIT1, OUTPUT);

  SPI.begin();
  SPI.setHwCs(0);
  SPI.beginTransaction(ATTinySPISettings);

  const uint32_t bits = (64 * 8) - 1;
  const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  SPI_BIT_COUNT_MASK = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));

  pinMode(STATUS_LED_PIN, OUTPUT);

  Serial.begin(250000, SERIAL_8N1);

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

  txInputDataBuffers[0] = new DataBuffer(txDataBufferArr[0][0], sizeof(txDataBufferArr[0][0]));
  txInputDataBuffers[1] = new DataBuffer(txDataBufferArr[1][0], sizeof(txDataBufferArr[1][0]));
  txInputDataBuffers[2] = new DataBuffer(txDataBufferArr[2][0], sizeof(txDataBufferArr[2][0]));
  txInputDataBuffers[3] = new DataBuffer(txDataBufferArr[3][0], sizeof(txDataBufferArr[3][0]));

  txOutputDataBuffers[0] = new DataBuffer(txDataBufferArr[0][1], sizeof(txDataBufferArr[0][1]));
  txOutputDataBuffers[1] = new DataBuffer(txDataBufferArr[1][1], sizeof(txDataBufferArr[1][1]));
  txOutputDataBuffers[2] = new DataBuffer(txDataBufferArr[2][1], sizeof(txDataBufferArr[2][1]));
  txOutputDataBuffers[3] = new DataBuffer(txDataBufferArr[3][1], sizeof(txDataBufferArr[3][1]));

  receiveDataBuffer[0] = new DataBuffer(receiveDataBufferArr[0], sizeof(receiveDataBufferArr[0]));
  receiveDataBuffer[1] = new DataBuffer(receiveDataBufferArr[1], sizeof(receiveDataBufferArr[1]));
  receiveDataBuffer[2] = new DataBuffer(receiveDataBufferArr[2], sizeof(receiveDataBufferArr[2]));
  receiveDataBuffer[3] = new DataBuffer(receiveDataBufferArr[3], sizeof(receiveDataBufferArr[3]));

  currentSPITransferUniverse = 0;

  delay(1000);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(1000);
}

void loop() {
  if (UDP.parsePacket()) {
    size_t read = UDP.read(udpBuffer, sizeof(udpBuffer));
    MyArtNet.onPacketReceived(UDP.remoteIP().v4(), UDP.remotePort(), udpBuffer, read);
  }

  if (micros() - lastDataReceived < 100000) {
    GPOS = GPOS | (1 << STATUS_LED_PIN);
  } else {
    GPOC = GPOC | (1 << STATUS_LED_PIN);
  }

  // Swap buffers if needed
  uint8_t universe = currentSPITransferUniverse;
  DataBuffer *txOutputBuffer = txOutputDataBuffers[universe];

  if (lastSendIndex[universe] == LAST_SEND_INDEX_SEND_NEXT_FRAME && txOutputBuffer->isEmpty()) {
    DataBuffer *txInputBuffer = txInputDataBuffers[universe];

    if (txInputBuffer->used() >= 192) {
      Serial.print("Swapping buffers....\n");
      lastSendIndex[universe] = 0;
      txOutputDataBuffers[universe] = txInputBuffer;
      txInputDataBuffers[universe] = txOutputBuffer;
      txOutputBuffer = txInputBuffer;
    }
  }

  // Download SPI data
  while(SPI1CMD & SPIBUSY) {}

  uint8_t count = 0;
  uint8_t* dataInPtr = (uint8_t*)&SPI1W0;
  DataBuffer *rxDataBuffer = receiveDataBuffer[universe];

  while (count < 64) {
    rxDataBuffer->enqueueIfValid((*dataInPtr));
    dataInPtr++;
    count++;
  }

  // Upload SPI data

  // transmit buffer
  // select chip

  // Bit 0
  /*
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
  */

  if (universe == 0) {
    GPOC = GPOC | (1 << CS_DEMULT_PIN_BIT0);
  } else {
    GPOS = GPOS | (1 << CS_DEMULT_PIN_BIT0);
  }

  count = 0;
  SPI1U1 = SPI_BIT_COUNT_MASK;
  dataInPtr = (uint8_t*)&SPI1W0;

  while (count < 64 && txOutputBuffer->used()) {
    (*dataInPtr) = txOutputBuffer->dequeue();
    count++;
    dataInPtr++;
  }

  while (count < 64) {
    (*dataInPtr) = 0;
    count++;
    dataInPtr++;
  }

  SPI1CMD |= SPIBUSY;

  while (rxDataBuffer->used() >= 2) {
    uint8_t data = rxDataBuffer->dequeue();

    if (data == DATA_CTRL_STUFF_BYTE) {
      uint8_t cmd = rxDataBuffer->dequeue();

      if (cmd == DATA_CTRL_RECEIVE_NEXT_FRAME) {
        Serial.print("Next frame request received;\n");
        if (micros() - lastDataReceived > 200000) {
          lastDataReceived = micros();
        }
        lastSendIndex[universe] = LAST_SEND_INDEX_SEND_NEXT_FRAME;
      }
    } 
  }

  currentSPITransferUniverse += 1;

  if (currentSPITransferUniverse > 3) {
    currentSPITransferUniverse = 0;
  }
}
