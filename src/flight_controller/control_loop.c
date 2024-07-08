#include "control_loop.h"

#include "nrf_port.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

#include "parameters.h"
#include "imu_sensors.h"
#include "imu.h"
#include "receivers.h"
#include "motors.h"
#include "PID.h"
#include "filters.h"
#include "quaternion_math.h"
#include "timer.h"


error_t clSetupQuadCopter(void)
{
    error_t errCode = SUCCESS;

    // Get gyro Data and set current angle using accelerometer.
    int16_t acc[3];
    vector3_t accData;
    errCode = imu_sensors_GetAccel(&accData);
    imu_fusion_CalcAngleFromAcc(&imu_fusion_GetCurrentAttitude()->pitch, &imu_fusion_GetCurrentAttitude()->roll, &accData);

    LOG("Setup Angles: Pitch: %i, Roll: %i \r\n",  (int32_t)imu_fusion_GetCurrentAttitude()->pitch, (int32_t)imu_fusion_GetCurrentAttitude()->roll);

    receiverEnable();

    return errCode;
}

void controlLoop(void)
{
    error_t retCode = 0;
    int32_t lastTime = 0; // Variable for storing last time executed.
    uint64_t loopCounter = 0;
    // Reset Variables
    // memset(&imu_fusion_GetCurrentAttitude(), 0, sizeof(attitude_t));

    static pidConfig_t pidConfigRoll = 
    {
      .maxI  = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_P, 
                 .i = PID_GAIN_I, 
                 .d = PID_GAIN_D},
    };
    static pidConfig_t pidConfigPitch = 
    {
      .maxI  = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_P, 
                 .i = PID_GAIN_I, 
                 .d = PID_GAIN_D},
    };
    static pidConfig_t pidConfigYaw = 
    {
      .maxI  = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_YAW_P, 
                 .i = PID_GAIN_YAW_I, 
                 .d = PID_GAIN_YAW_D},
    };

    // Setup
    clSetupQuadCopter();

    #if 0
    float hardErr[3];
    float SoftErr[3];
    retCode = imu_sensors_imuSensorsMagHardSoftCalibration(hardErr, SoftErr, 3500);
    ERROR_CHECK(retCode);
    #endif
    
    // Calibrate IMU
    retCode = imu_sensors_mpuGyroCalibration(500);
    ERROR_CHECK(retCode);

    float deltaT = 0;

    // Enter main loop.
    for (;;)
    { 
        int32_t dtInUs = 0, currentTime = 0; 
        while(dtInUs < MAIN_LOOP_MAX_TIME)
        {
            currentTime = getTimeUs();
            dtInUs = get_16bit_diff_tick(currentTime, lastTime);
        }
        lastTime = currentTime;
        deltaT = US_TO_SECONDS((float)dtInUs); // Calculate Loop time in seconds
                
        // Update sensors
        imu_fusion_UpdateSensors(NULL);
  
        // Update attitude
        imu_fusion_UpdateAttitude(deltaT);

        // Test quaternion algorithm
        // Calculate PID for each axis
        float rollOutput  = pidCalculate(&pidConfigRoll,  imu_fusion_GetCurrentAttitude()->vRate.x, (imu_fusion_GetCurrentAttitude()->vRateSP.x), PID_MAX_OUTPUT);
        float pitchOutput = pidCalculate(&pidConfigPitch, imu_fusion_GetCurrentAttitude()->vRate.y, (imu_fusion_GetCurrentAttitude()->vRateSP.y), PID_MAX_OUTPUT);
        float yawOutput   = pidCalculate(&pidConfigYaw,   imu_fusion_GetCurrentAttitude()->vRate.z, (imu_fusion_GetCurrentAttitude()->vRateSP.z), PID_MAX_OUTPUT);
                
        receivers_t rc = rcGetChannels();
        // Mix outputs to esc values
        motors_t esc = motorsMix(rc.throttle, pitchOutput, rollOutput, yawOutput, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
        
        bool updateMotors = false;
        // If enabled change motor pulse
        switch (rcSwitchGet3Way())
        {
            case RC_SWITCH_3WAY_OFF:              
                motorsUpdateAll(1000);
                updateMotors = false;
                break;
            case RC_SWITCH_3WAY_OFF_TO_ON:
                LOG("RC_SWITCH_3WAY_OFF_TO_ON\r\n");
                // Reset PID parameters to avoid integral windup when starting
                pidReset(&pidConfigPitch);
                pidReset(&pidConfigRoll);
                pidReset(&pidConfigYaw);                

                vector3_t acc;
                imu_sensors_GetAccel(&acc);
                imu_fusion_CalcAngleFromAcc(&imu_fusion_GetCurrentAttitude()->pitch, &imu_fusion_GetCurrentAttitude()->roll, &acc);

                break;
            case RC_SWITCH_3WAY_ON_TO_OFF:
                LOG("RC_SWITCH_3WAY_ON_TO_OFF\r\n");
                break;
            case RC_SWITCH_3WAY_ON:
                updateMotors = true;
                break;
            case RC_SWITCH_3WAY_ONPLUS:
                updateMotors = false;
                break;
            default:
                updateMotors = false;
                break;
        }
        
        if (updateMotors == true)
        {
            motorsUpdate(esc.esc1, esc.esc2, esc.esc3, esc.esc4); // Front Right - Front Left - Back Right - Back Left
        }
        else
        {
            motorsUpdateAll(1000);
        }

        //Print variables every 50 loop cycles
        if (!(loopCounter % 50))
        {        
          utilPrintFloatArray("Raw Gyro  :", imu_fusion_GetCurrentAttitude()->vRate.v, 3); 

          float eulersAngles[3] = {imu_fusion_GetCurrentAttitude()->roll, imu_fusion_GetCurrentAttitude()->pitch, imu_fusion_GetCurrentAttitude()->yaw};
          utilPrintFloatArray("Angles PRY:", eulersAngles, 3);

          utilPrintFloatArray("Set p  PRY:", imu_fusion_GetCurrentAttitude()->vRateSP.v, 3);

          float receivers[6] = {rc.throttle, rc.pitch, rc.roll, rc.yaw};
          utilPrintFloatArray("RC T PRY  :", receivers, 4);

          float pidOutput[3] = {rollOutput, pitchOutput, yawOutput};
          utilPrintFloatArray("Pid Out   :", pidOutput, 3);

          float escs[4] = {esc.esc1,esc.esc2,esc.esc3,esc.esc4};
          utilPrintFloatArray("ESCs      :", escs, 4); 

          LOG("Loop Time: %i \r\n",dtInUs);
        }
        FLUSH();
        loopCounter++;
    }
}


#if 0
volatile uint64_t t2IntCounter = 0;
/** TIMER2 Interrupt at 4 ms. Used to Service motors */
void TIMER2_IRQHandler(void)
{
  if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;  //Clear compare register 0 event	

      

        if (!(t2IntCounter % 150))
        {            
            //LOG("Timer2 Int time: %i \r\n", usTimer);
            //FLUSH();
        }
        t2IntCounter++;
  }
}

static void timer2_init()
{

    // PreScaler 8 and 62500 for 1s f
    NRF_TIMER2->MODE        = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
    NRF_TIMER2->PRESCALER   = 4;                          
    NRF_TIMER2->BITMODE     = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->CC[0]       = 4000;  // 4000 For interrupt every 4ms
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER2->TASKS_START = 1;  // Start event generation.
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    NVIC_EnableIRQ(TIMER2_IRQn);

}
#endif




void controlLoopInit(void)
{

    // Start Program
    controlLoop();
}

#include "nrf_strerror.h"
#include "nrf_sdm.h"
#include "app_error.h"

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_FINAL_FLUSH();

#ifndef DEBUG
    NRF_LOG_ERROR("Fatal error");
#else
    switch (id)
    {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            NRF_LOG_ERROR("ERROR %u [%s] at %s:%u",
                          p_info->err_code,
                          nrf_strerror_get(p_info->err_code),
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        default:
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
    }
#endif

    //motorsUpdateAll(1000);
    motorsDisable();
    //NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.

#ifndef DEBUG
    NRF_LOG_WARNING("System reset");
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}

#include "hardfault.h"

void HardFault_process(HardFault_stack_t * p_stack)
{
    //motorsUpdateAll(1000);


    motorsDisable();

    // Restart the system by default
    NVIC_SystemReset();
}