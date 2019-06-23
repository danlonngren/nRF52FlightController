#include "setup_board.h"
#include "board_config.h"

#include "nrf_port.h"

#include "nrf_drv_gpiote.h"

#include "twi.h"

error_t setupBoardInit(void)
{

    twiInit(SCL_PIN, SDA_PIN, 8);

    nrf_gpio_cfg_input(PIN_RECEIVER_CH4_11, NRF_GPIO_PIN_PULLDOWN);

    nrf_gpio_cfg_output(PIN_ESC1_A0);
    nrf_gpio_cfg_output(PIN_ESC2_A1);
    nrf_gpio_cfg_output(PIN_ESC3_A2);
    nrf_gpio_cfg_output(PIN_ESC4_A3);

    nrf_gpio_cfg_output(PIN_LED1_19);
    nrf_gpio_cfg_output(PIN_LED1_17);

    //twiInit(SCL_PIN, SDA_PIN, 8);

    return SUCCESS;
}