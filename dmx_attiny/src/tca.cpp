#include <Arduino.h>
#include <clock-custom.h>
#include <tca.h>

ISR(TCA0_OVF_vect) {
  if (TCA0_SINGLE_INTFLAGS & 0x1) {
    tcaHasOverflow = 1;
    TCA0_SINGLE_INTFLAGS = 0x1;
  }
}

void tcaPrepare() {
  TCA0_SINGLE_INTCTRL = 0x1;

  // 12 ms every ovf
  TCA0_SINGLE_PER = (uint16_t)60000U;
  TCA0_SINGLE_CTRLD = 0;
  TCA0_SINGLE_CTRLC = 0;
  TCA0_SINGLE_CTRLB = TCA_SINGLE_ALUPD_bm;
}

void tcaStart() {
  TCA0_SINGLE_CTRLA = 0;
  TCA0_SINGLE_CNT = (uint16_t)0;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL0_bm;
}

void tcaStop() {
  TCA0_SINGLE_CTRLA = 0;
}