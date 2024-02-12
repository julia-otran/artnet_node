#include <Arduino.h>
#include <clock-custom.h>
#include <tca.h>

ISR(TCA0_OVF_vect) {
  if (TCA0_SINGLE_INTFLAGS & 0x1) {
    if (tcaOverflows == 0xFF) {
      tcaOverflows = 0;
    } else {
      tcaOverflows++;
    }

    TCA0_SINGLE_INTFLAGS = 0x1;
  }
}

void tcaStart() {
  TCA0_SINGLE_INTCTRL = 0x1;

  // 12 ms every ovf
  TCA0_SINGLE_PER = TCA_PERIOD;

  TCA0_SINGLE_CTRLD = 0;
  TCA0_SINGLE_CTRLC = 0;
  TCA0_SINGLE_CTRLB = 0;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL0_bm;
}

micro_t tcaMicros() {
  uint16_t us = TCA0_SINGLE_CNT / (uint16_t)5;

  // 1 ovf = 60,000 OSC / 5 OSC / us
  // 1 ovf = 12,000 us
  uint32_t usOvf = tcaOverflows * 12000U;

  return us + usOvf;
}

micro_t tcaMicrosDelta(micro_t previous) {
  micro_t current = tcaMicros();

  if (current >= previous) {
    return current - previous;
  }

  return (TCA_MAX_MICROS - previous) + current;
}
