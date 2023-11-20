#include <Arduino.h>
#include <SPI.h>
#include <buffer.h>

// DMX_TX -> PB2 - #7
#define DMX_TX_BREAK_PIN PIN_PA5 // #3

// DMX_RX -> PB3 - #6
#define DMX_RX_CABLE_DISCONNECT_PIN PIN_PA4 // #2

int currentOutputDmxChannel;
unsigned long dmxOutputBreakStartAt;

uint8_t writeBufferMemo[512];
DmxBuffer DmxTxBuffer(writeBufferMemo, 512);

// SPI MOSI  -> PA1 - #11
// SPI MISO  -> PA2 - #12
// SPI CLOCK -> PA3 - #13

#define MOSI_bp 1
#define MOSI_bm (1 << MOSI_bp)
#define MISO_bp 2
#define MISO_bm (1 << MISO_bp)
#define SCK_bp 3
#define SCK_bm (1 << SCK_bp)
#define SS_bp 4
#define SS_bm (1 << SS_bp)

uint8_t spiDmxOutputShouldBreak;
uint8_t spiDmxOutputBreakPosition;
uint8_t spiDataReady;
int8_t spiDataPosition;
uint16_t spiDataInputFlags;

#define DEVICE_UNIVERSE_ID 0

#define SPI_DATA_START_BIT (1 << 7)
#define SPI_DATA_WRITE_BITS (3 << 1)

uint8_t getSpiUniverseDataMSB(uint8_t universeId) {
  return (spiDataInputFlags >> (12 + universeId)) & 1;
}

uint8_t getSpiUniverseHasData(uint8_t universeId) {
  return spiDataInputFlags & (1 << (3 + (universeId * 2)));
}

uint8_t getSpiUniverseHasBreak(uint8_t universeId) {
  return spiDataInputFlags & (1 << (4 + (universeId * 2)));
}

uint8_t getSpiDmxDataPosition() {
  uint8_t position = 0;

  for (uint8_t universeId = 0; universeId < DEVICE_UNIVERSE_ID; universeId++) {
    if (getSpiUniverseHasData(universeId)) {
      position++;
    }
  }

  return position;
}

uint8_t canWriteToSPI() {
  return ((spiDataInputFlags >> 8) & SPI_DATA_WRITE_BITS) == (DEVICE_UNIVERSE_ID << 1);
}

void spiReadData() {
  if(SPI0.INTFLAGS & SPI_RXCIF_bm) {
    uint8_t data = SPI0.DATA;

    if (data & SPI_DATA_START_BIT) {
      spiDataPosition = 0;
      spiDataReady = 0;
    }

    if (spiDataPosition == 0) {
      spiDataInputFlags = ((uint16_t)data) << 8;
      spiDataPosition++;
    } else if (spiDataPosition == 1) {
      spiDataInputFlags = spiDataInputFlags | data;
      spiDataPosition++;

      if (getSpiUniverseHasBreak(DEVICE_UNIVERSE_ID) && dmxOutputBreakStartAt == 0) {
        spiDmxOutputBreakPosition = DmxTxBuffer.used();
        spiDmxOutputShouldBreak = 1;
      }
    } else if (getSpiUniverseHasData(DEVICE_UNIVERSE_ID) && spiDataPosition == getSpiDmxDataPosition()) {
      uint8_t spiDataDmxValue = data | (getSpiUniverseDataMSB(DEVICE_UNIVERSE_ID) << 7);

      if (DmxTxBuffer.canEnqueue()) {
        DmxTxBuffer.enqueue(spiDataDmxValue);
      }

      spiDataPosition = -1;
      spiDataReady = 1;
    } else if (spiDataPosition > 1 && spiDataPosition < 6) {
      spiDataPosition++;
    }
  }
}

void dmxTxData() {
  if (DmxTxBuffer.used() && Serial.availableForWrite() > 0) {
    if (spiDmxOutputShouldBreak == 0 || spiDmxOutputBreakPosition > 0) {
      Serial.write(DmxTxBuffer.dequeue());
      spiDmxOutputBreakPosition--;
    }
  }

  if (spiDmxOutputBreakPosition == 0 && spiDmxOutputShouldBreak && dmxOutputBreakStartAt == 0) {
    uint8_t usedBytes = SERIAL_TX_BUFFER_SIZE - Serial.availableForWrite();

    if (usedBytes <= 1) {
      Serial.flush();
      pinModeFast(DMX_TX_BREAK_PIN, OUTPUT);
      digitalWriteFast(DMX_TX_BREAK_PIN, LOW);
      dmxOutputBreakStartAt = micros();
    }
  }

  if (dmxOutputBreakStartAt > 0 && (micros() - dmxOutputBreakStartAt) >= 88) {
    pinModeFast(DMX_TX_BREAK_PIN, INPUT);
    dmxOutputBreakStartAt = 0;
    spiDmxOutputShouldBreak = 0;
  }
}

void setup() {
  spiDataPosition = -1;
  spiDataReady = 0;
  currentOutputDmxChannel = -1;
  dmxOutputBreakStartAt = 0;
  spiDmxOutputBreakPosition = 0;
  spiDmxOutputShouldBreak = 0;
  
  PORTA.DIRSET = MISO_bm;
  SPI0.CTRLB = SPI_MODE_2_gc;
  SPI0.CTRLA = !SPI_MASTER_bm | // as Slave
                SPI_ENABLE_bm; // Start

  pinMode(DMX_TX_BREAK_PIN, INPUT);
  pinMode(DMX_RX_CABLE_DISCONNECT_PIN, INPUT);

  Serial.begin(250000, SERIAL_8N2);
}

void loop() {
  spiReadData();
  dmxTxData();
}