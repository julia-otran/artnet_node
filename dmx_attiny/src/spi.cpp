#include <Arduino.h>
#include <spi.h>

#define MOSI_bp 1
#define MOSI_bm (1 << MOSI_bp)
#define MISO_bp 2
#define MISO_bm (1 << MISO_bp)
#define SCK_bp 3
#define SCK_bm (1 << SCK_bp)
#define SS_bp 4
#define SS_bm (1 << SS_bp)

void spiStart() {
  PORTA.DIRCLR = 1 << 4;
  PORTA.DIRSET = MISO_bm;
  SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm | 1;
  SPI0.INTCTRL = 0;
  SPI0.CTRLA = SPI_ENABLE_bm; // Start
}
