/*
 * MPU6050.c
 *
 * Created: 15/06/2019 14:38:25
 *  Author: Danlo
 */ 

#include "motors.h"

#include "nrf_gpio.h"

#include "board_config.h"

#include "nrf_port.h"

#include "nrf_drv_pwm.h"

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t top_value = 2000;

// Declare variables holding PWM sequence values. In this example only one channel is used 
nrf_pwm_values_individual_t seq_values[] = {1000, 1000, 1000, 1000};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

#if NOT_IN_USE
void motorPwmHandlers(nrf_drv_pwm_evt_type_t event_type)
{

    switch (event_type)
    {
        case NRF_DRV_PWM_EVT_FINISHED: ///< Sequence playback finished.
          //  LOG("NRF_DRV_PWM_EVT_FINISHED\r\n");
            break;
        case NRF_DRV_PWM_EVT_END_SEQ0: /**< End of sequence 0 reached. Its data can be safely modified now. */
            LOG("NRF_DRV_PWM_EVT_END_SEQ0\r\n");
            break;
        case NRF_DRV_PWM_EVT_END_SEQ1: /**< End of sequence 1 reached. Its data can be safely modified now. */
            LOG("NRF_DRV_PWM_EVT_END_SEQ1\r\n");
            break;
        case NRF_DRV_PWM_EVT_STOPPED:  ///< The PWM peripheral has been stopped.
            LOG("NRF_DRV_PWM_EVT_STOPPED\r\n");
            break;
        default:
            break;

    }
    FLUSH();
}
#endif


// Set duty cycle between 0 and 100%
void pwm_update_duty_cycle(uint16_t dc1, uint16_t dc2, uint16_t dc3, uint16_t dc4)
{
    // Check if value is outside of range. If so, set to 100%
    if(dc1 >= top_value)
    {
        seq_values->channel_0 = 1;
        seq_values->channel_1 = 1;
        seq_values->channel_2 = 1;
        seq_values->channel_3 = 1;
    }
    else
    {
        seq_values->channel_0 = top_value - dc1;
        seq_values->channel_1 = top_value - dc2;
        seq_values->channel_2 = top_value - dc3;
        seq_values->channel_3 = top_value - dc4;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_STOP);
}

// Set duty cycle between 0 and 2000%
void pwm_update_duty_cycle_all(uint16_t dc)
{
    // Check if value is outside of range. If so, set to 100%
    pwm_update_duty_cycle(dc, dc, dc, dc);
}


esc_t motorsMix(float throttle, float pitch, float roll, float yaw, float minOut, float maxOut)
{
    esc_t escOut;

    escOut.esc1 = throttle - pitch - roll - yaw;//(front-right - CCW)
    if (escOut.esc1 < minOut) escOut.esc1 = minOut;
    else if (escOut.esc1 > maxOut) escOut.esc1 = maxOut;

    escOut.esc2 = throttle - pitch + roll + yaw;//(rear-right - CW)
    if (escOut.esc2 < minOut) escOut.esc2 = minOut;
    else if (escOut.esc2 > maxOut) escOut.esc2 = maxOut;

    escOut.esc3 = throttle + pitch + roll - yaw;//(rear-left - CCW)
    if (escOut.esc3 < minOut) escOut.esc3 = minOut;
    else if (escOut.esc3 > maxOut) escOut.esc3 = maxOut;

    escOut.esc4 = throttle + pitch - roll + yaw;//(front-left - CW)
    if (escOut.esc4 < minOut) escOut.esc4 = minOut;
    else if (escOut.esc4 > maxOut) escOut.esc4 = maxOut;

    return escOut;
}


error_t motorsDisable(void)
{
    error_t retCode = nrf_drv_pwm_stop(&m_pwm0, false);
    return retCode;
}


error_t motorsPwmSetup(void)
{
   error_t err_code = SUCCESS;
   nrf_drv_pwm_config_t const config0 =
    {
        .output_pins = {PIN_ESC1_A0, PIN_ESC2_A1, PIN_ESC3_A2, PIN_ESC4_A3},
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = top_value,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_TRIGGERED, // NRF_PWM_STEP_AUTO,
    };
    // Init PWM without error handler
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);

    return SUCCESS;
}
 

error_t motorsSetup(void)
{

    error_t errCode = motorsPwmSetup();

    pwm_update_duty_cycle(1000, 1000, 1000, 1000); 

    return errCode;
}