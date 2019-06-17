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




#if 0
static uint64_t currentTime = 0;
static uint64_t LastTime = 0;
uint32_t reloadT = 25000 * 2;
/** TIMTER2 peripheral interrupt handler. This interrupt handler is called whenever there it a TIMER2 interrupt
 */
void TIMER2_IRQHandler(void)
{
  if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) && 
     ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;           //Clear compare register 0 event	

        //nrf_gpio_pin_set(GPIO_TOGGLE_PIN);           //Set LED
        LOG("dTIme: %i \r\n", currentTime - LastTime);
        FLUSH();
        LastTime = currentTime;
  }
}


static void timer2_init()
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
    // PreScaler 8 and 62500 for 1s f
    NRF_TIMER2->MODE        = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
    NRF_TIMER2->PRESCALER   = 8;                          
    NRF_TIMER2->BITMODE     = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->CC[0]       = 625;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER2->TASKS_START = 1;  // Start event generation.
    //NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    //NVIC_EnableIRQ(TIMER2_IRQn);
}
#endif


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


#define PWM_CH_MAX  9 // Quanum 8ch + 1 to allow for detect of sync
volatile uint32_t pwmChannels[PWM_CH_MAX];


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


volatile uint32_t channelCount = 0;
volatile uint64_t startT = 0;
volatile uint64_t prevT = 0;

void GPIOTE_IRQHandler(void)
{
    if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        startT = NRF_TIMER3->CC[0];

        if (channelCount < PWM_CH_MAX)
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

static error_t rc_pwm_gpioe_init(void)
{
    nrf_gpio_cfg_input(PIN_RECEIVER_CH4_11, NRF_GPIO_PIN_PULLUP);

    NVIC_EnableIRQ(GPIOTE_IRQn);

    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos | 
                                         GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos | 
                                         PIN_RECEIVER_CH4_11 << GPIOTE_CONFIG_PSEL_Pos | 
                                         GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;    // No effect in Event mode.
    
    NRF_GPIOTE->EVENTS_IN[0] = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos;
    
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[0];
    NRF_PPI->CHENSET = (1 << 0);

    return SUCCESS;
}


// This function initializes timer 3 with the following configuration:
// 24-bit, base frequency 16 MHz.
error_t rc_pwm_timer3_init()
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }
    NRF_TIMER3->BITMODE                 = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->PRESCALER               = 4;
    NRF_TIMER3->MODE                    = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER3->TASKS_START = 1;  // Start event generation.

    return SUCCESS;
}


error_t receiverSetup(void)
{
    error_t errCode = SUCCESS;

    errCode = rc_pwm_gpioe_init();

    errCode = rc_pwm_timer3_init();

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



#if 0
uint32_t out_pins[] = {5, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED};
static uint16_t pwm_seq[2] = {0x8800, 0x0800};
nrf_pwm_sequence_t const seq = 
{
    .values.p_common = pwm_seq,
    .length          = sizeof(pwm_seq)/sizeof(uint16_t),
    .repeats         = 0,
    .end_delay       = 0
};

nrf_gpio_cfg_output(5);
nrf_pwm_pins_set(NRF_PWM0, out_pins);
nrf_pwm_enable(NRF_PWM0);
nrf_pwm_configure(NRF_PWM0, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, 1600);
nrf_pwm_loop_set(NRF_PWM0, 0);
nrf_pwm_decoder_set(NRF_PWM0, NRF_PWM_LOAD_COMMON, NRF_PWM_STEP_AUTO);

nrf_pwm_sequence_set(NRF_PWM0, 0, &seq);

NRF_PWM0->TASKS_SEQSTART[0] = 1;
#endif