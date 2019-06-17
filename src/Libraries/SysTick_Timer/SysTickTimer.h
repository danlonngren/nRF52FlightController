#ifndef SYSTICKTIMER_H
#define SYSTICKTIMER_H


#include "error_codes.h"


#define SYSTIC_FREQUENCY        10UL
//#define SYSTIC_CONVERTER        (SystemCoreClock / (SYSTIC_FREQUENCY * 1000.0))
#define SYSTIC_CONVERTER        64UL //(SystemCoreClock / (1000UL * 1000UL)) // per ms
#define SYSTIC_OVERFLOW_COUNT   (SystemCoreClock / SYSTIC_FREQUENCY)
#define SYSTIC_MS_TO_TICKS(x)   (x * SYSTIC_CONVERTER * 1000UL)
#define SYSTIC_US_TO_TICKS(x)   (x * SYSTIC_CONVERTER)
#define SYSTIC_TICKS_TO_US(x)   ((x) / SYSTIC_CONVERTER)
#define SYSTIC_TICKS_TO_MS(x)   ((x) / SYSTIC_CONVERTER * 1000UL)


error_t sysTickTimerInit(void);

uint64_t sysTickGetMillis(void);

uint64_t sysTickGetMicros(void);

uint64_t stsTickCompTime(uint64_t CurrentTime, uint64_t LastTime);

uint32_t sysTickGet(void);

uint32_t sysTickCompTick(uint32_t tickNow, uint32_t lastTick);

#endif