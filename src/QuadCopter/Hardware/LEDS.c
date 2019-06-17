/*
 * MPU6050.c
 *
 * Created: 15/06/2019 14:38:25
 *  Author: Danlo
 */ 

#include "LEDS.h"

#include "nrf_drv_gpiote.h"

#include "board_config.h"

void ledBlueSetState(bool state)
{
  if (state)
  {
      nrf_gpio_pin_set(PIN_LED1_19);
  }
  else
  {
      nrf_gpio_pin_clear(PIN_LED1_19);
  }
}


void ledRedSetState(bool state)
{
  if (state)
  {
      nrf_gpio_pin_set(PIN_LED1_17);
  }
  else
  {
      nrf_gpio_pin_clear(PIN_LED1_17);
  }
}
