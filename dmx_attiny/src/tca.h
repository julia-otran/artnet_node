#include <clock-custom.h>

static uint8_t tcaOverflows;

#define TCA0_CLK (CLK_PER / 2U)
#define TCA_PERIOD ((uint16_t) ((CLK_PER / 1000U) * 6U))
#define TCA_MAX_OVERFLOW_COUNT 256U
#define TCA_OVERFLOW_MICROS 12000U
#define TCA_MAX_MICROS (TCA_MAX_OVERFLOW_COUNT * TCA_OVERFLOW_MICROS)

typedef uint32_t micro_t;

void tcaStart();

micro_t tcaMicros();
micro_t tcaMicrosDelta(micro_t previous);

#define TCA_MICROS (tcaMicros())



