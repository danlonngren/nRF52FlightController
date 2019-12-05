#include "setup_board.h"
#include "board_config.h"

#include "nrf_port.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_clock.h"

#include "twi.h"

#include "timer.h"

#include "log.h"

#define LOG_LEVEL_DEFAULT    ELOG_LEVEL_DEBUG


error_t setupBoardInit(void)
{

    //-----------------------------------------------------------------------------
    // LF Clock setup
    //-----------------------------------------------------------------------------
    // Start lfcl, Used for RTC and scheduler
    error_t err_code = nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }

    //-----------------------------------------------------------------------------
    // Logging Setup
    //-----------------------------------------------------------------------------
    static sLogSetup logInit = {
      .pFncPrintf = printf,
      .pSrcDirName = "src",
    };
    for (uint32_t i = 0; i < VLOG_SYS_SIZE; i++)
    {
        logInit.subSysLevel[i] = LOG_LEVEL_DEFAULT;
    }

    // Setup NRF Logging module
    vLogInit(&logInit);

    //-----------------------------------------------------------------------------
    // Comms and GPIO setup
    //-----------------------------------------------------------------------------
    twiInit(SCL_PIN, SDA_PIN, 8);

    nrf_gpio_cfg_input(PIN_RECEIVER_CH4_11, NRF_GPIO_PIN_PULLDOWN);

    nrf_gpio_cfg_output(PIN_ESC1_A0);
    nrf_gpio_cfg_output(PIN_ESC2_A1);
    nrf_gpio_cfg_output(PIN_ESC3_A2);
    nrf_gpio_cfg_output(PIN_ESC4_A3);

    nrf_gpio_cfg_output(PIN_LED1_19);
    nrf_gpio_cfg_output(PIN_LED1_17);

    //twiInit(SCL_PIN, SDA_PIN, 8);

    timer0Setup();

    return SUCCESS;
}