
#include "nrf_port.h"

#include "ble_backend.h"

#include "nrf_delay.h"

#include "nrf_systick.h"

#include "error_codes.h"

#include "log.h"
VLOG_MODULE_INIT("main", vLOG_LEVEL_DEBUG);

/*
<warning> nrf_sdh_ble: RAM starts at 0x200020E0, can be adjusted to 0x200020F0.
<warning> nrf_sdh_ble: RAM size can be adjusted to 0xDF10.

FLASH_SIZE=0x5d000;RAM_START=0x200020e0;RAM_SIZE=0xdf20"
*/


#include "control_loop.h"
#include "imu_sensors.h"
#include "setup_board.h"
#include "LEDS.h"
#include "receivers.h"
#include "motors.h"

#include "twi.h"

#include "FuzzyLogic.h"

#define TEST_MODE 1

/**@brief Function for application main entry.
 */
int main(void)
{
    nrf_delay_ms(500);

    // Initialize.
    ERROR_CHECK(setupBoardInit());

    ERROR_CHECK(MPU6050_init());
    
    ERROR_CHECK(receiverSetup());

    ERROR_CHECK(motorsSetup());

    //ledBlueSetState(1);

    ledRedSetState(true);
    #if TEST_MODE
    uint32_t results = fuzzyTest();
    LOG("FuzzyTest Results: %i \r\n", results);
    #else
    controlLoopInit();
    #endif

    // Enter main loop.
    for (;;)
    { 
    }
} /*** End main ***/


