#include "control_loop.h"

#include "nrf_port.h"

#include "imu_sensors.h"

#include "quad_dynamics.h"

#include "app_util_platform.h"

#include "nrf_delay.h"

#include "receivers.h"

#include "motors.h"

#include "PID.h"

#include "filters.h"

#include "quaternion_math.h"

#include "parameters.h"

int16_t getTimeUs(void)
{
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    uint16_t temp = NRF_TIMER0->CC[0];
    return temp;
}


typedef struct
{


  rcSwitch3Way_t switch3LastState;

}clDataStruct_t;

static attitude_t qAttitude; /*! <Structure Containing QuadCopter angles> */

clDataStruct_t cl = {

  .switch3LastState = RC_SWITCH_3WAY_OFF,
};

error_t clSetupQuadCopter(void)
{
    error_t errCode = SUCCESS;

    // Get gyro Data and set current angle using accelerometer.
    int16_t acc[3];

    errCode = mpuReadAccXYZ(acc);

    qdCalculateAngleFromAcc(&qAttitude.pitch, &qAttitude.roll, (float)acc[0], (float)acc[1], (float)acc[2]);

    LOG("Setup Angles: Pitch: %i, Roll: %i \r\n",  (int32_t)qAttitude.pitch, (int32_t)qAttitude.roll);

    receiverEnable();

    return errCode;
}



axis_t qData;

void controlLoop(void)
{
    error_t retCode = 0;
    int32_t lastTime = 0; // Variable for storing last time executed.
    uint64_t loopCounter = 0;
    // Reset Variables
    memset(&qAttitude, 0, sizeof(attitude_t));

    static pidConfig_t pidConfigRoll = 
    {
      .maxI = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_P, .i = PID_GAIN_I, .d = PID_GAIN_D},
    };
    static pidConfig_t pidConfigPitch = 
    {
      .maxI = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_P, .i = PID_GAIN_I, .d = PID_GAIN_D},
    };
    static pidConfig_t pidConfigYaw = 
    {
      .maxI = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_YAW_P, .i = PID_GAIN_YAW_I, .d = PID_GAIN_YAW_D},
    };

    // Setup
    clSetupQuadCopter();

    #if 0
    float hardErr[3];
    float SoftErr[3];
    retCode = imuSensorsMagHardSoftCalibration(hardErr, SoftErr, 3500);
    ERROR_CHECK(retCode);
    #endif
    
    // Calibrate IMU
    retCode = mpuGyroCalibration(500);
    ERROR_CHECK(retCode);
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
        qAttitude.dt = US_TO_SECONDS((float)dtInUs); // Calculate Loop time in seconds

        float gyroRateR[3];
        float accG[3];
        float magT[3];

        imuSensorGetData(gyroRateR, accG, magT);
        magT[0] /= 1000.0f; // Convert to uG from mG
        magT[1] /= 1000.0f; // Convert to uG from mG
        magT[2] /= 1000.0f; // Convert to uG from mG
        
        static float gyroRateFilter[3];
        gyroRateFilter[0] = FILTER_COMP(toDegrees(gyroRateR[0]), gyroRateFilter[0], FILTER_COMP_GAIN_GYRO_RATE);
        gyroRateFilter[1] = FILTER_COMP(toDegrees(gyroRateR[1]), gyroRateFilter[1], FILTER_COMP_GAIN_GYRO_RATE);
        gyroRateFilter[2] = FILTER_COMP(toDegrees(gyroRateR[2]), gyroRateFilter[2], FILTER_COMP_GAIN_GYRO_RATE);

        static float accGFilter[3];
        accGFilter[0] = FILTER_COMP(accG[0], accGFilter[0], FILTER_COMP_GAIN_ACC);
        accGFilter[1] = FILTER_COMP(accG[1], accGFilter[1], FILTER_COMP_GAIN_ACC);
        accGFilter[2] = FILTER_COMP(accG[2], accGFilter[2], FILTER_COMP_GAIN_ACC);
      
        #if 1
        // Calculate angles and quad-copter attitude.
        qdCalculateAngle(&qAttitude, gyroRateFilter, accGFilter);
             
        // Calculate Set points from rc inputs
        receivers_t rc = rcGetChannels();
        qdCalculateSetPoints(&qAttitude, rc.pitch, rc.roll, rc.yaw, RECEIVER_LEVEL_ADJUST_GAIN, SET_POINT_DIV, true);

        static float setPointFiltered[3];
        setPointFiltered[0] = FILTER_COMP(qAttitude.pitchSet, setPointFiltered[0], 0.3f);
        setPointFiltered[1] = FILTER_COMP(qAttitude.rollSet,  setPointFiltered[1], 0.3f);
        setPointFiltered[2] = FILTER_COMP(qAttitude.yawSet,   setPointFiltered[2], 0.3f);

        // Calculate PID for each axis
        float pitchOutput = pidCalculate(&pidConfigPitch, gyroRateFilter[1], setPointFiltered[0], PID_MAX_OUTPUT);
        float rollOutput  = pidCalculate(&pidConfigRoll,  gyroRateFilter[0], setPointFiltered[1], PID_MAX_OUTPUT);
        float yawOutput   = pidCalculate(&pidConfigYaw,   gyroRateFilter[2], setPointFiltered[2], PID_MAX_OUTPUT);
                
        // Mix outputs to esc values
        esc_t esc = motorsMix(rc.throttle, pitchOutput, rollOutput, yawOutput, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
        
        // If enabled change motor pulse
        switch (rcSwitchGet3Way())
        {
            case RC_SWITCH_3WAY_OFF:              
                pwm_update_duty_cycle_all(1000);
                break;
            case RC_SWITCH_3WAY_OFF_TO_ON:
                LOG("RC_SWITCH_3WAY_OFF_TO_ON\r\n");
                // Reset PID parameters to avoid integral windup when starting
                pidReset(&pidConfigPitch);
                pidReset(&pidConfigRoll);
                pidReset(&pidConfigYaw);                
                break;
            case RC_SWITCH_3WAY_ON_TO_OFF:
                LOG("RC_SWITCH_3WAY_ON_TO_OFF\r\n");
                break;
            case RC_SWITCH_3WAY_ON:
                pwm_update_duty_cycle(esc.esc1, esc.esc2, esc.esc3, esc.esc4); // Front Right - Front Left - Back Right - Back Left
                break;
            case RC_SWITCH_3WAY_ONPLUS:
                //pwm_update_duty_cycle(esc.esc1, esc.esc2, esc.esc3, esc.esc4); // Front Right - Front Left - Back Right - Back Left
                break;
            default:
                LOG("ERROR: Switch 3 Undefined \r\n");
                pwm_update_duty_cycle_all(1000);
                break;
        }
#endif
  
        #if 0
        qData.acc.x  = accG[0];
        qData.acc.y  = accG[1];
        qData.acc.z  = accG[2];
        qData.gyro.x = gyroRateR[0];
        qData.gyro.y = gyroRateR[1];
        qData.gyro.z = gyroRateR[2];
        qData.mag.x  = magT[0];
        qData.mag.y  = magT[1];
        qData.mag.z  = magT[2];
        
        // Test quaternion algorithm
        // mQuaternionTest(qData, qAttitude.dt);
        #endif

        //Print variables every 50 loop cycles
        if (!(loopCounter % 50))
        {        
          utilPrintFloatArray("Raw Gyro  :", gyroRateFilter, 3); 

         // utilPrintFloatArray("Raw Acc   :", accGFilter, 3);

          //utilPrintFloatArray("Raw Mag   :", magT, 3);

          float angles[3] = {qAttitude.pitch, qAttitude.roll,qAttitude.yaw};
          utilPrintFloatArray("Angles PRY:", angles, 3);

          float rpy[3] = {qAttitude.pitchSet, qAttitude.rollSet,qAttitude.yawSet};
          utilPrintFloatArray("Set p  PRY:", setPointFiltered, 3);

          float receivers[6] = {rc.throttle, rc.pitch, rc.roll, rc.yaw};
          utilPrintFloatArray("RC T PRY  :", receivers, 4);

          float pidOutput[3] = {pitchOutput, rollOutput, yawOutput};
          utilPrintFloatArray("Pid Out   :", pidOutput, 3);

          float escs[4] = {esc.esc1,esc.esc2,esc.esc3,esc.esc4};
          utilPrintFloatArray("ESCs      :", escs, 4); 
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
    NRF_TIMER2->CC[0]       = 2000;  // 4000 For interrupt every 4ms
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER2->TASKS_START = 1;  // Start event generation.
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    NVIC_EnableIRQ(TIMER2_IRQn);

}
#endif

static void timer0Setup(void)
{
    NRF_TIMER0->MODE        = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
    NRF_TIMER0->PRESCALER   = 4;                          
    NRF_TIMER0->BITMODE     = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER0->TASKS_START = 1;  // Start event generation.

}

void controlLoopInit(void)
{
    timer0Setup();

 
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

    //pwm_update_duty_cycle_all(1000);
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
    //pwm_update_duty_cycle_all(1000);


    motorsDisable();

    // Restart the system by default
    NVIC_SystemReset();
}