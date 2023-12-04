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
#define DATA_CTRL_RECEIVE_NEXT_FRAME 0xF1
#define DATA_CTRL_1BYTE_DATA 0xE0
#define DATA_CTRL_2BYTE_DATA 0xE1
#define DATA_CTRL_1BYTE_ZERO_DATA 0xD0
#define DATA_CTRL_2BYTE_ZERO_DATA 0xD1

// We should always run at 10Mhz
#define CLK_PER (10UL * 1000UL * 1000UL)

#define DMX_OUTPUT_BREAK_PIN PIN_PA5

#define SERIAL_TRANSMIT_POSITION_UNKNOWN -1

unsigned long dmxOutputBreakStartedAt;
unsigned long dmxOutputInactiveAt;
int16_t dmxTransmitPosition;
uint8_t dmxTransmitBreak;
uint8_t transmitDmxBufferArr[500];
DataBuffer TransmitDmxBuffer(transmitDmxBufferArr, 500);

uint8_t spiReceivedPreviousData;
uint8_t transmitSpiBufferArr[256];
DataBuffer TransmitSpiBuffer(transmitSpiBufferArr, 256);
unsigned long lastTransmitBufferEmpty;

uint32_t tcaOverflows;

ISR(TCA0_OVF_vect) {
  if (TCA0_SINGLE_INTFLAGS & 0x1) {
    if (tcaOverflows == 0xFFFF) {
      tcaOverflows = 0;
    } else {
      tcaOverflows++;
    }

    TCA0_SINGLE_INTFLAGS = 0x1;
  }
}

unsigned long tcaMillis() {
  return (tcaOverflows * 6UL) + (TCA0_SINGLE_CNT / 10000UL);
}

unsigned long tcaMicros() {
  return (tcaOverflows * 6UL * 1000UL) + (TCA0_SINGLE_CNT / 10UL);
}

void setup() {
  // Makes tiny run at 10Mhz (20Mhz OSC / 2)
  // Set clock divider to 2.
  // Be sure that:
  // 1. VDD is at least 3V. Lower voltages can cause problems
  // 2. Fuse is set to 20Mhz. (Factory default)
  CPU_CCP = 0xD8;
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

  dmxTransmitPosition = SERIAL_TRANSMIT_POSITION_UNKNOWN;
  dmxTransmitBreak = 0;
  dmxOutputBreakStartedAt = 0;
  dmxOutputInactiveAt = 0;

  pinMode(DMX_OUTPUT_BREAK_PIN, INPUT);

  // Serial lib is buggy. Ignore this baud rate.
  Serial.begin(250000, SERIAL_8N2);

  // Fix baud rate
  // convert baud rate to register value
  int32_t baud_reg_val = (4 * CLK_PER) / 250000;

  // adjust baud rate according to OSC error rate
  baud_reg_val *= (1024 + sigrowVal);
  baud_reg_val /= 1024;

  USART0.BAUD = (int16_t) baud_reg_val;

  PORTA.DIRSET = MISO_bm;
  SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm | 1;
  SPI0.CTRLA = SPI_ENABLE_bm; // Start

  TCA0_SINGLE_INTCTRL = 0x1;
  // 6 ms every ovf
  TCA0_SINGLE_PER = (uint16_t) ((CLK_PER / 1000UL) * 6UL);

  TCA0_SINGLE_CTRLD = 0;
  TCA0_SINGLE_CTRLC = 0;
  TCA0_SINGLE_CTRLB = 0;
  TCA0_SINGLE_CTRLA = 0x1;
}

void loop() {
  if (TransmitSpiBuffer.used() > 0 && (SPI0.INTFLAGS & SPI_DREIF_bm)) {
    SPI0.DATA = TransmitSpiBuffer.dequeue();
  }

  if(SPI0.INTFLAGS & SPI_RXCIF_bm) {
    uint8_t data = SPI0.DATA;

    if (data > 0) {
      if (spiReceivedPreviousData == DATA_CTRL_STUFF_BYTE) {
        if (data == DATA_CTRL_TRANSMISSION_BEGIN) {
          dmxTransmitPosition = 0;
          dmxTransmitBreak = 1;
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

  if (dmxTransmitPosition == SERIAL_TRANSMIT_POSITION_UNKNOWN) {
    TransmitDmxBuffer.clear();
  } else if (dmxTransmitPosition >= 513) {
    dmxTransmitPosition = SERIAL_TRANSMIT_POSITION_UNKNOWN;
    TransmitDmxBuffer.clear();
  } else if (dmxTransmitBreak > 0) {
    unsigned long currentMicros = tcaMicros();

    if (SERIAL_TX_BUFFER_SIZE - Serial.availableForWrite() <= 1 && dmxTransmitBreak == 1) {
      Serial.flush();
      pinModeFast(DMX_OUTPUT_BREAK_PIN, OUTPUT);
      dmxOutputBreakStartedAt = currentMicros;
      dmxTransmitBreak = 2;
    }
    
    unsigned long timeDelta = currentMicros - dmxOutputBreakStartedAt;

    if (dmxTransmitBreak == 2 && timeDelta >= 88UL) {
      pinModeFast(DMX_OUTPUT_BREAK_PIN, INPUT);
      dmxTransmitBreak = 3;
      dmxOutputBreakStartedAt = currentMicros;
    }

    if (dmxTransmitBreak == 3 && timeDelta >= 8UL) {
        dmxTransmitBreak = 0;
        dmxOutputBreakStartedAt = 0;
    }
  } else if (TransmitDmxBuffer.used() > 0) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(TransmitDmxBuffer.dequeue());
      dmxTransmitPosition++;
    }

    dmxOutputInactiveAt = 0;
    lastTransmitBufferEmpty = 0;
  } else if (SERIAL_TX_BUFFER_SIZE - Serial.availableForWrite() <= 1) {
    if (dmxOutputInactiveAt == 0) {
      dmxOutputInactiveAt = tcaMicros();
    } else if (tcaMicros() - dmxOutputInactiveAt >= 92) {
      dmxTransmitPosition = SERIAL_TRANSMIT_POSITION_UNKNOWN;
      dmxOutputInactiveAt = 0;
    }
  }

  if (TransmitDmxBuffer.isEmpty()) {
    if (TransmitSpiBuffer.available() >= 2) {
      if (tcaMillis() - lastTransmitBufferEmpty >= 30UL) {
        TransmitSpiBuffer.enqueue(DATA_CTRL_STUFF_BYTE);
        TransmitSpiBuffer.enqueue(DATA_CTRL_RECEIVE_NEXT_FRAME);
        lastTransmitBufferEmpty = tcaMillis();
      }
    }
  }
}
