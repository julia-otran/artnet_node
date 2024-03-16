#include <Arduino.h>
#include <clock-custom.h>
#include <buffer.h>
#include <tca.h>
#include <spi.h>
#include <serial.h>

uint8_t transmitDmxBufferArr[768];
DataBuffer TransmitDmxBuffer(transmitDmxBufferArr, sizeof(transmitDmxBufferArr));
uint8_t clearSwitch;

void setup() {
  // Makes tiny run at 10Mhz (20Mhz OSC / 2)
  // Set clock divider to 2.
  // Be sure that:
  // 1. VDD is at least 3V. Lower voltages can cause problems
  // 2. Fuse is set to 20Mhz. (Factory default)
  CPU_CCP = 0xD8;
  CLKCTRL_MCLKCTRLB = 1;

  uint8_t bodLevel = BOD.CTRLB & 0b00000111;

  if (bodLevel == 0x2) {
    // 20Mhz OSC error at 3V (BOD Level 2)
    sigrowVal = SIGROW.OSC20ERR3V;
  } else if (bodLevel == 0x7) {
    // 20Mhz OSC error at 5V (BOD Level 7)
    sigrowVal = SIGROW.OSC20ERR5V;
  }

  PORTB_DIRCLR = 1;

  clearSwitch = 0;

  serialStart();
  spiStart();
}

void loop() {
  if (SPI0_INTFLAGS & SPI_DREIF_bm) {
    SPI0_DATA = 0;
  }

  if (SPI0_INTFLAGS & SPI_RXCIF_bm) {
    TransmitDmxBuffer.enqueue(SPI0_DATA);
  }

  if (USART0_STATUS & USART_DREIF_bm) {
    if (TransmitDmxBuffer.used()) {
      USART0_TXDATAL = TransmitDmxBuffer.dequeue();
    }
  }

  if (PORTB_IN & 1) {
    if (clearSwitch == 0) {
      PORTA_DIRCLR = 1 << 5;
      TransmitDmxBuffer.clear();
      clearSwitch = 1;
    }
  } else {
    if (clearSwitch == 1) {
      clearSwitch = 0;
      PORTA_DIRSET = 1 << 5;
    }
  }
}
