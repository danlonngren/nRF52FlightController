/*
 * MPU6050.c
 *
 * Created: 15/06/2019 14:38:25
 *  Author: Danlo
 */ 

#include "receivers.h"

#include "nrf_gpio.h"

#include "board_config.h"

#include "nrf_port.h"

#include "nrf_delay.h"


#define PWM_CH_MAX  9 // Quanum 8ch + 1 to allow for detect of sync
volatile uint32_t pwmChannels[PWM_CH_MAX] = {1500, 1500, 1000, 1500, 1500, 1500, 1500};
volatile uint32_t channelCount = 0;
volatile uint64_t startT = 0;
volatile uint64_t prevT = 0;

uint16_t get_16bit_diff_tick(uint16_t test_tick, uint16_t prev_tick)
{
    if (test_tick < prev_tick)
    {
        return (0xFFFF - prev_tick) + 1 + test_tick;
    }
    else
    {
        return test_tick - prev_tick;
    }
}



receivers_t rcGetChannels(void)
{
  receivers_t rc = 
  {
    .roll       = pwmChannels[0],
    .pitch      = pwmChannels[1],
    .throttle   = pwmChannels[2],
    .yaw        = pwmChannels[3],
  };
  return rc;
}

uint32_t rc_get_roll(void)
{
   return pwmChannels[0];
}

uint32_t rc_get_pitch(void)
{
   return pwmChannels[1];
}

uint32_t rc_get_throttle(void)
{
    return pwmChannels[2];
}

uint32_t rc_get_yaw(void)
{
   return pwmChannels[3];
}

uint32_t rc_get_6_way_switch(void)
{
    return pwmChannels[4];
}

uint32_t rc_get_3_way_switch(void)
{
    return pwmChannels[5];
}

rcSwitch3Way_t rcSwitchGet3Way(void)
{ 
    rcSwitch3Way_t stateOut = RC_SWITCH_3WAY_OFF;
    uint16_t switch3WayVal = rc_get_3_way_switch();
    if (switch3WayVal < 1900)
    {
        if (switch3WayVal < 1400)
        {
            stateOut = RC_SWITCH_3WAY_ON;
        }
        else
        {
            stateOut = RC_SWITCH_3WAY_ONPLUS;
        }
    }
    else if (switch3WayVal > 1900)
    { 
        stateOut = RC_SWITCH_3WAY_OFF;
    }
    return stateOut;
}


void GPIOTE_IRQHandler(void)
{
    if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        startT = NRF_TIMER3->CC[0];

        if (channelCount < PWM_CH_MAX && prevT != 0)
        {
            pwmChannels[channelCount] = get_16bit_diff_tick(startT, prevT);
            
            if (pwmChannels[channelCount] > 2100)
            {
                pwmChannels[channelCount] = 0;
                channelCount = 0;
            }
            else
            {
                channelCount++;
            }
        }
        prevT = startT;
    }
}

static error_t rc_pwm_init_gpioe(void)
{
    nrf_gpio_cfg_input(PIN_RECEIVER_CH4_11, NRF_GPIO_PIN_PULLUP);

    NVIC_EnableIRQ(GPIOTE_IRQn);

    NRF_GPIOTE->CONFIG[CH_GPIOTE] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos | 
                                         GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos | 
                                         PIN_RECEIVER_CH4_11 << GPIOTE_CONFIG_PSEL_Pos | 
                                         GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;    // No effect in Event mode.
    
    NRF_GPIOTE->EVENTS_IN[CH_GPIOTE] = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos;
    
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[CH_GPIOTE];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[CH_TIMER];
    NRF_PPI->CHENSET = (1 << 0);

    return SUCCESS;
}


// This function initializes timer 3 with the following configuration:
// 24-bit, base frequency 16 MHz.
error_t rc_pwm_init_timer3()
{

    NRF_TIMER3->BITMODE                   = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->PRESCALER                 = 4;
    NRF_TIMER3->MODE                      = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->TASKS_CLEAR               = 1;
    NRF_TIMER3->EVENTS_COMPARE[CH_TIMER]  = 0;
    NRF_TIMER3->SHORTS                    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER3->TASKS_START               = 1;

    return SUCCESS;
}


error_t receiverSetup(void)
{
    error_t errCode = SUCCESS;

    errCode = rc_pwm_init_gpioe();

    errCode = rc_pwm_init_timer3();

    #if RECEIVER_DEBUG_LOG
    while(1)
    {
      uint8_t i = 0;
      for (i = 0; i < 8; i++)
      {
         LOG("PWM %i: %i \r\n",i,  pwmChannels[i]);
         FLUSH();
         nrf_delay_ms(1);
      }
      nrf_delay_ms(100);
    }
    #endif

    return errCode;
}
