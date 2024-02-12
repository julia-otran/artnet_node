#include <Arduino.h>
#include <serial.h>
#include <clock-custom.h>

void serialStart() {
  PORTB_DIRSET = 1 << 2;
  PORTB_DIRCLR = 1 << 1;

  USART0_CTRLA = 0;
  USART0_CTRLC = USART_CMODE0_bm | USART_SBMODE_2BIT_gc | USART_CHSIZE0_bm | USART_CHSIZE1_bm;
  USART0_CTRLB = USART_TXEN_bm;
}
