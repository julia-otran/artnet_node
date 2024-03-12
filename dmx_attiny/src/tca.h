#include <clock-custom.h>

static uint8_t tcaHasOverflow;

#define TCA0_CLK (CLK_PER / 2U)
#define TCA_PERIOD ((uint16_t) ((CLK_PER / 1000U) * 6U))
#define TCA_MAX_OVERFLOW_COUNT 256U
#define TCA_OVERFLOW_MICROS 12000U
#define TCA_MAX_MICROS (TCA_MAX_OVERFLOW_COUNT * TCA_OVERFLOW_MICROS)

void tcaPrepare();
void tcaStart();
void tcaStop();
