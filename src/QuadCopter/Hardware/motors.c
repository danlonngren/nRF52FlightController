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

#include "nrf_delay.h"

#include "nrf_drv_pwm.h"



error_t motorsPwmSetup(void)
{
    uint32_t out_pins[] = {PIN_ESC1_A0, PIN_ESC2_A1, PIN_ESC3_A2, PIN_ESC4_A3};
    static uint16_t pwm_seq[] = {1000};
    nrf_pwm_sequence_t const seq = {
        .values.p_common = pwm_seq,
        .length          = sizeof(pwm_seq)/sizeof(uint16_t),
        .repeats         = 0,
        .end_delay       = 0
    };


    nrf_pwm_pins_set(NRF_PWM0, out_pins);
    nrf_pwm_enable(NRF_PWM0);
    nrf_pwm_configure(NRF_PWM0, NRF_PWM_CLK_1MHz, NRF_PWM_MODE_UP, 2000);
    nrf_pwm_loop_set(NRF_PWM0, 0);
    nrf_pwm_decoder_set(NRF_PWM0, NRF_PWM_LOAD_COMMON, NRF_PWM_STEP_AUTO);

    nrf_pwm_sequence_set(NRF_PWM0, 0, &seq);

    NRF_PWM0->TASKS_SEQSTART[0] = 1;

    return SUCCESS;
}



error_t motorsSetup(void)
{

// Already Set in setup_board.c/h
//    nrf_gpio_cfg_output(PIN_ESC1_A0);
//    nrf_gpio_cfg_output(PIN_ESC2_A1);
//    nrf_gpio_cfg_output(PIN_ESC3_A2);
//    nrf_gpio_cfg_output(PIN_ESC4_A3);

    error_t errCode = SUCCESS;
    errCode = motorsPwmSetup();
    return errCode;
}