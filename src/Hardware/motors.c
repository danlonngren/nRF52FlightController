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

static nrf_drv_pwm_t nrf_pwm_instance0 = NRF_DRV_PWM_INSTANCE(0);


motorsConfig_t motorsConfig = {
    .topValueCount = MOTORS_PWM_TOP_VALUES,
};


nrf_drv_pwm_config_t const config0 = {
    .output_pins  = {PIN_ESC1_A0, PIN_ESC2_A1, PIN_ESC3_A2, PIN_ESC4_A3},
    .irq_priority = APP_IRQ_PRIORITY_LOWEST,
    .base_clock   = NRF_PWM_CLK_1MHz,
    .count_mode   = NRF_PWM_MODE_UP,
    .top_value    = MOTORS_PWM_TOP_VALUES,
    .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    .step_mode    = NRF_PWM_STEP_TRIGGERED, // NRF_PWM_STEP_AUTO,
};


nrf_pwm_values_individual_t seq_values[] = {1000, 1000, 1000, 1000};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};


// Set duty cycle between 0 and 100%
void pwm_update_duty_cycle(uint16_t dc1, uint16_t dc2, uint16_t dc3, uint16_t dc4)
{
    // Check if value is outside of range. If so, set to 100%
    if(dc1 >= motorsConfig.topValueCount)
    {
        seq_values->channel_0 = 1;
        seq_values->channel_1 = 1;
        seq_values->channel_2 = 1;
        seq_values->channel_3 = 1;
    }
    else
    {
        seq_values->channel_0 = motorsConfig.topValueCount - dc1;
        seq_values->channel_1 = motorsConfig.topValueCount - dc2;
        seq_values->channel_2 = motorsConfig.topValueCount - dc3;
        seq_values->channel_3 = motorsConfig.topValueCount - dc4;
    }
    
    nrf_drv_pwm_simple_playback(&nrf_pwm_instance0, &seq, 1, NRF_DRV_PWM_FLAG_STOP);
}


motors_t motorsMix(float throttle, float pitch, float roll, float yaw, float minOut, float maxOut)
{
    motors_t escOut;
    escOut.esc1 = throttle - pitch - roll - yaw;  // (front-right - CCW)
    escOut.esc2 = throttle - pitch + roll + yaw;  // (rear-right - CW)
    escOut.esc3 = throttle + pitch + roll - yaw;  // (rear-left - CCW)
    escOut.esc4 = throttle + pitch - roll + yaw;  // (front-left - CW)

    for (int32_t index = 0; index < 4; index++)
    {
        if (escOut.esc[index] < minOut) 
        {
            escOut.esc[index] = minOut;
        }
        else if (escOut.esc[index] > maxOut) 
        {
            escOut.esc[index] = maxOut;
        }
    }
    return escOut;
}


void motorsUpdate(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
    pwm_update_duty_cycle(motor1, motor2, motor3, motor4);
}


// Set duty cycle between 0 and 2000%
void motorsUpdateAll(uint16_t dc)
{
    // Check if value is outside of range. If so, set to 100%
    motorsUpdate(dc, dc, dc, dc);
}


error_t motorsDisable(void)
{
    error_t retCode = nrf_drv_pwm_stop(&nrf_pwm_instance0, false);
    return retCode;
}


error_t motorsSetup(void)
{
    // Init PWM without error handler
    error_t err_code = nrf_drv_pwm_init(&nrf_pwm_instance0, &config0, NULL);
    ERROR_CHECK(err_code);

    motorsUpdate(1000, 1000, 1000, 1000); 
}
