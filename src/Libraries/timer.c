#include "timer.h"

#include "nrf.h"


int16_t getTimeUs(void)
{
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    uint16_t temp = NRF_TIMER0->CC[0];
    return temp;
}


void timer0Setup(void)
{
    NRF_TIMER0->MODE              = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
    NRF_TIMER0->PRESCALER         = 4;                          
    NRF_TIMER0->BITMODE           = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
    NRF_TIMER0->TASKS_CLEAR       = 1;
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->SHORTS            = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER0->TASKS_START       = 1;  // Start event generation.
}