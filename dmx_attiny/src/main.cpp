#include <Arduino.h>
#include <SPI.h>
#include <buffer.h>

#define MOSI_bp 1
#define MOSI_bm (1 << MOSI_bp)
#define MISO_bp 2
#define MISO_bm (1 << MISO_bp)
#define SCK_bp 3
#define SCK_bm (1 << SCK_bp)
#define SS_bp 4
#define SS_bm (1 << SS_bp)

#define DATA_CTRL_STUFF_BYTE 0xCA
#define DATA_CTRL_TRANSMISSION_BEGIN 0xF0
#define DATA_CTRL_TRANSMISSION_END 0xF1
#define DATA_CTRL_1BYTE_DATA 0xE0
#define DATA_CTRL_2BYTE_DATA 0xE1
#define DATA_CTRL_1BYTE_ZERO_DATA 0xD0
#define DATA_CTRL_2BYTE_ZERO_DATA 0xD1

// We should always run at 10Mhz
#define CLK_PER (10 * 1000 * (uint32_t)1000)

#define DMX_OUTPUT_BREAK_PIN PIN_PA5

#define SERIAL_TRANSMIT_POSITION_UNKNOWN -1
#define SERIAL_TRANSMIT_POSITION_BREAK -2

long dmxOutputBreakStartedAt;
long dmxOutputInactiveAt;
int16_t serialTransmitPosition;
uint8_t transmitDmxBufferArr[512];
DataBuffer TransmitDmxBuffer(transmitDmxBufferArr, 512);

uint8_t spiReceivedPreviousData;
uint8_t transmitSpiBufferArr[512];
DataBuffer TransmitSpiBuffer(transmitSpiBufferArr, 512);

void setup() {
  // Makes tiny run at 10Mhz (20Mhz OSC / 2)
  // Set clock divider to 2.
  // Be sure that:
  // 1. VDD is at least 3V. Lower voltages can cause problems
  // 2. Fuse is set to 20Mhz. (Factory default)
  CLKCTRL_MCLKCTRLB = 1;

  uint8_t bodLevel = BOD.CTRLB & 0b00000111;
  uint8_t sigrowVal;

  if (bodLevel == 0x2) {
    // 20Mhz OSC error at 3V (BOD Level 2)
    sigrowVal = SIGROW.OSC20ERR3V;
  } else if (bodLevel == 0x7) {
    // 20Mhz OSC error at 5V (BOD Level 7)
    sigrowVal = SIGROW.OSC20ERR5V;
  }

  serialTransmitPosition = SERIAL_TRANSMIT_POSITION_UNKNOWN;
  dmxOutputBreakStartedAt = 0;
  dmxOutputInactiveAt = 0;

  pinMode(DMX_OUTPUT_BREAK_PIN, INPUT);

  // Serial lib is buggy. Ignore this baud rate.
  Serial.begin(250000, SERIAL_8N2);

  // Fix baud rate
  // convert baud rate to register value
  int32_t baud_reg_val = (8 * CLK_PER) / 250000;

  // adjust baud rate according to OSC error rate
  baud_reg_val *= (1024 + sigrowVal);
  baud_reg_val /= 1024;

  USART0.BAUD = (int16_t) baud_reg_val;

  PORTA.DIRSET = MISO_bm;
  SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm;
  SPI0.CTRLA = SPI_ENABLE_bm; // Start
}

void loop() {
  if(SPI0.INTFLAGS & SPI_RXCIF_bm) {
    uint8_t data = SPI0.DATA;

    if (TransmitSpiBuffer.used() > 0) {
      uint8_t sendData = TransmitSpiBuffer.get();

      if (SPI0.INTFLAGS & SPI_DREIF_bm) {
        SPI0.DATA = sendData;
        TransmitSpiBuffer.dequeue();
      }
    }

    if (data > 0) {
      if (spiReceivedPreviousData == DATA_CTRL_STUFF_BYTE) {
        if (data == DATA_CTRL_TRANSMISSION_BEGIN) {
          serialTransmitPosition = SERIAL_TRANSMIT_POSITION_BREAK;
        } else if (data == DATA_CTRL_1BYTE_DATA) {
          TransmitDmxBuffer.enqueue(DATA_CTRL_STUFF_BYTE);
        } else if (data == DATA_CTRL_2BYTE_DATA) {
          TransmitDmxBuffer.enqueue(DATA_CTRL_STUFF_BYTE);
          TransmitDmxBuffer.enqueue(DATA_CTRL_STUFF_BYTE);
        } else if (data == DATA_CTRL_1BYTE_ZERO_DATA) {
          TransmitDmxBuffer.enqueue(0);
        } else if (data == DATA_CTRL_2BYTE_ZERO_DATA) {
          TransmitDmxBuffer.enqueue(0);
          TransmitDmxBuffer.enqueue(0);
        }
      } else if (data != DATA_CTRL_STUFF_BYTE) {
        TransmitDmxBuffer.enqueue(data);
      }

      spiReceivedPreviousData = data;
    }
  }

  if (serialTransmitPosition == SERIAL_TRANSMIT_POSITION_UNKNOWN) {
    TransmitDmxBuffer.dequeue();
  } else if (serialTransmitPosition == SERIAL_TRANSMIT_POSITION_BREAK) {
    if (SERIAL_TX_BUFFER_SIZE - Serial.availableForWrite() <= 1 && dmxOutputBreakStartedAt == 0) {
      Serial.flush();
      pinModeFast(DMX_OUTPUT_BREAK_PIN, OUTPUT);
      dmxOutputBreakStartedAt = micros();
    } else if (dmxOutputBreakStartedAt > 0 && micros() - dmxOutputBreakStartedAt >= 88) {
      pinModeFast(DMX_OUTPUT_BREAK_PIN, INPUT);

      if (micros() - dmxOutputBreakStartedAt >= 96) {
        serialTransmitPosition = 0;
        dmxOutputBreakStartedAt = 0;
      }
    }
  } else {
    if (serialTransmitPosition == 0) {
      Serial.write(0);
      serialTransmitPosition++;
    }

    if (TransmitDmxBuffer.used() > 0) {
      if (serialTransmitPosition >= 513) {
        TransmitDmxBuffer.dequeue();
      } else if (Serial.availableForWrite() > 0) {
        Serial.write(TransmitDmxBuffer.dequeue());
        serialTransmitPosition++;
      }
    }

    if (SERIAL_TX_BUFFER_SIZE - Serial.availableForWrite() <= 1) {
      if (dmxOutputInactiveAt == 0) {
        dmxOutputInactiveAt = micros();
      } else if (micros() - dmxOutputInactiveAt >= 88) {
        serialTransmitPosition = SERIAL_TRANSMIT_POSITION_UNKNOWN;
        dmxOutputInactiveAt = 0;
      }
    } else {
      dmxOutputInactiveAt = 0;
    }
  }

  if (serialTransmitPosition >= 513 && TransmitSpiBuffer.available() >= 2) {
    serialTransmitPosition = SERIAL_TRANSMIT_POSITION_UNKNOWN;
    TransmitSpiBuffer.enqueue(DATA_CTRL_STUFF_BYTE);
    TransmitSpiBuffer.enqueue(DATA_CTRL_TRANSMISSION_END);
  }
}
