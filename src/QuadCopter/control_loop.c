#include "control_loop.h"

#include "nrf_port.h"

#include "MPU6050.h"

#include "quad_dynamics.h"

#include "app_util_platform.h"

#include "SysTickTimer.h"

#include "nrf_delay.h"

void controlLoop(void)
{
    uint64_t clCounter = 0;
    uint64_t dtInUs = 0;
    uint64_t lastUs = 0;
    uint64_t currentUs = 0;
    
    // Reset Variables
    attitude_t qAttitude = {
      .pitch  = 0,
      .roll   = 0,
      .yaw    = 0,
    };

    MPU6050_data_t mpuData = MPU6050_getData();
    qdCalculateAngleFromAcc(&qAttitude.pitch, &qAttitude.roll, mpuData.acc.x, mpuData.acc.y, mpuData.acc.z);

    LOG("Started: Pitch: %i, Roll: %i \r\n",  (int32_t)qAttitude.pitch, (int32_t)qAttitude.roll);
    FLUSH();

    // Enter main loop.
    for (;;)
    { 
        clCounter++;

        lastUs = currentUs;
        currentUs = sysTickGetMicros();
        dtInUs = stsTickCompTime(currentUs, lastUs);

        qAttitude.dt = ((float)dtInUs / 1000.0f); // 4 ms
        MPU6050_data_t mpuData = MPU6050_getData();
        qdCalculateAttitude(&qAttitude, mpuData.gyro.x, mpuData.gyro.y, mpuData.gyro.z, mpuData.acc.x, mpuData.acc.y, mpuData.acc.z);
    
 
        if (!(clCounter % 100))
        {
            LOG("%i: Pitch: %i, Roll: %i, Yaw: %i :  \r\n", (int32_t)dtInUs, (int32_t)qAttitude.pitch, (int32_t)qAttitude.roll, (int32_t)qAttitude.yaw);
            FLUSH();
        }
        nrf_delay_ms(2);
    }
}

void controlLoopInit(void)
{
    controlLoop();
}
