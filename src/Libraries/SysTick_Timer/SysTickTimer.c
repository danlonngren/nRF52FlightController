#include "SysTickTimer.h"

#include "nrf_systick.h"

#include "app_util_platform.h"

#include "nrf_port.h"

#include <math.h>

volatile uint64_t g_systick_val = 0;


void SysTick_Handler(void) 
{
    // TODO: Handle Overflow.
   // SysTick->LOAD  = (uint32_t)(SYSTIC_OVERFLOW_COUNT - 1UL);
    g_systick_val += (SYSTIC_OVERFLOW_COUNT - 1);
    SysTick->VAL   = 0UL; 
}

uint64_t lastsysTickVal = 0;
uint64_t lastsysTickcnt = 0;

uint64_t sysTickGetMicros(void)
{
    uint64_t out = 0;
    
    uint64_t sysTickVal = SYSTIC_OVERFLOW_COUNT;
    uint64_t sysTickOvf = 0;
    CRITICAL_REGION_ENTER();
    sysTickVal -= nrf_systick_val_get();
    sysTickOvf = g_systick_val;
    if ((sysTickVal < lastsysTickVal) && (sysTickOvf == lastsysTickcnt))
    {
        //sysTickVal += ((SYSTIC_OVERFLOW_COUNT) - lastsysTickVal);
        LOG("OVERFLOW\r\n");
    }
    else
    {
        lastsysTickVal = sysTickVal;
        lastsysTickcnt = sysTickOvf;
    }
    CRITICAL_REGION_EXIT();
    out = SYSTIC_TICKS_TO_US(sysTickVal + sysTickOvf);
    
    return out;
}


uint64_t stsTickCompTime(uint64_t CurrentTime, uint64_t LastTime)
{
    uint64_t dt = CurrentTime - LastTime;
    if (CurrentTime < LastTime)
    {
      LOG("OverFlow \r\n");
      return 0;
    }
    else
    {
      return dt;
    }
}

uint64_t sysTickGetMillis(void)
{
    return sysTickGetMicros() / 1000UL;
}


uint32_t sysTickGet(void)
{
  uint32_t out = SYSTIC_OVERFLOW_COUNT - 1;
  CRITICAL_REGION_ENTER();
  out -= nrf_systick_val_get();
  CRITICAL_REGION_EXIT();
  return out;
}

uint32_t sysTickCompTick(uint32_t tickNow, uint32_t lastTick)
{
    uint32_t dt = 0;
    if (tickNow < lastTick)
    {
       dt = tickNow + (SYSTIC_OVERFLOW_COUNT - lastTick);
    }
    else
    {
       dt = (tickNow - lastTick);
    }
    return dt;
}

#include "nrf_delay.h"

error_t sysTickTimerInit(void)
{
    
    SysTick_Config(SYSTIC_OVERFLOW_COUNT);
    NVIC_SetPriority( SysTick_IRQn, 7 );
    NVIC_EnableIRQ(SysTick_IRQn);
    return SUCCESS;
}