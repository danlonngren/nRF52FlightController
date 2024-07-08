
#include "nrf_port.h"
#include "nrf_log_default_backends.h"


#include "ble_backend.h"
#include "nrf_drv_clock.h"

#include "nrf_delay.h"

#include "nrf_systick.h"

#include "error_codes.h"

#include "control_loop.h"
#include "imu_sensors.h"
#include "setup_board.h"
#include "LEDS.h"
#include "receivers.h"
#include "motors.h"

#include "twi.h"

#include "timer.h"


/*
<warning> nrf_sdh_ble: RAM starts at 0x200020E0, can be adjusted to 0x200020F0.
<warning> nrf_sdh_ble: RAM size can be adjusted to 0xDF10.

FLASH_SIZE=0x5d000;RAM_START=0x200020e0;RAM_SIZE=0xdf20"
*/

error_t chip_startup_init(void)
{
    error_t err_code;
    // Setup NRF Logging module
    err_code = NRF_LOG_INIT(NULL);
    
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Start lfcl, Used for RTC and scheduler
    err_code = nrf_drv_clock_init();

    nrf_drv_clock_lfclk_request(NULL);

        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }

    return err_code;
}


void thread_log_flush(void * p_context)
{

    while(1)
    {
        if (NRF_LOG_PROCESS())
        {
            FLUSH();
        }
        nrf_delay_ms(1);
        //taskSuspend(&log_flush_task);
    }
} /*** End thread_log_flush ***/


/**@brief Function for application main entry.
 */
int main(void)
{
    nrf_delay_ms(500);

    // Initialize.
    ERROR_CHECK(chip_startup_init());

    ERROR_CHECK(setupBoardInit());

    timer0Setup();

    ERROR_CHECK(imu_sensors_MPU6050_init());
    
    ERROR_CHECK(receiverSetup());

    ERROR_CHECK(motorsSetup());

    //ledBlueSetState(1);


    ledRedSetState(true);

    controlLoopInit();

    // Enter main loop.
    for (;;)
    { 

    }
} /*** End main ***/


