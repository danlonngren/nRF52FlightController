#include "control_loop.h"

#include "nrf_port.h"

#include "MPU6050.h"

#include "quad_dynamics.h"

#include "app_util_platform.h"

#include "SysTickTimer.h"

#include "nrf_delay.h"

#include "receivers.h"

#include "motors.h"

#include "PID.h"

#include "filters.h"


int16_t getTimeUs(void)
{
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    uint16_t temp = NRF_TIMER0->CC[0];
    return temp;
}


#define PID_GAIN_P 0.70f
#define PID_GAIN_I 0.0007f
#define PID_GAIN_D 7.2f

#define PID_GAIN_YAW_P 1.8f
#define PID_GAIN_YAW_I 0.02f
#define PID_GAIN_YAW_D 0.0f

#define PID_MAX_I_ERROR 150.0f

static attitude_t qAttitude; /*! <Structure Containing QuadCopter angles> */

error_t clSetupQuadCopter(void)
{
    error_t errCode = SUCCESS;

    // Get gyro Data and set current angle using accelerometer.
    MPU6050_data_t mpuData = MPU6050_getData();
    qdCalculateAngleFromAcc(&qAttitude.pitch, &qAttitude.roll, mpuData.acc.x, mpuData.acc.y, mpuData.acc.z);

    LOG("Setup Angles: Pitch: %i, Roll: %i \r\n",  (int32_t)qAttitude.pitch, (int32_t)qAttitude.roll);

    return errCode;
}


uint16_t actualTimeTaken = 0;


typedef struct
{
  uint16_t dtInUs;
  uint16_t lastT;
  uint16_t actualTimeTaken;
  uint16_t currentTime;
  uint64_t loopCounter;

  rcSwitch3Way_t switch3LastState;
  const uint16_t maxLoopTimeUs;
  const uint16_t maxEscOutput;
  const uint16_t minEscOutput;
  const uint16_t maxPidOutput;

  const float setPointDiv;
  const float gainLevelAdjust;
  const float gainCompFilter;
}clDataStruct_t;

clDataStruct_t cl = {
  .maxEscOutput  = 2000,
  .minEscOutput  = 1050,
  .maxPidOutput  = 400,
  .maxLoopTimeUs = 3000,
  
  .setPointDiv = 2.0f,
  .gainLevelAdjust = 15.0f, 
  .gainCompFilter  = 0.7f,

  .actualTimeTaken = 0,
  .loopCounter = 0,
  .currentTime = 0,
  .dtInUs = 0,
  .lastT = 0,
  .switch3LastState = RC_SWITCH_3WAY_OFF,
};

void controlLoop(void)
{
    // Reset Variables
    memset(&qAttitude, 0, sizeof(attitude_t));

    static pidConfig_t pidRoll = 
    {
      .maxI = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_P, .i = PID_GAIN_I, .d = PID_GAIN_D},
    };
    static pidConfig_t pidPitch = 
    {
      .maxI = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_P, .i = PID_GAIN_I, .d = PID_GAIN_D},
    };
    static pidConfig_t pidYaw = 
    {
      .maxI = PID_MAX_I_ERROR,
      .gains = { .p = PID_GAIN_YAW_P, .i = PID_GAIN_YAW_I, .d = PID_GAIN_YAW_D},
    };

    // Setup
    clSetupQuadCopter();

    // Enter main loop.
    for (;;)
    { 
      
        cl.actualTimeTaken = get_16bit_diff_tick(getTimeUs(), cl.lastT);
        cl.dtInUs = 0;
        while(cl.dtInUs < cl.maxLoopTimeUs)
        {
            cl.currentTime = getTimeUs();
            cl.dtInUs = get_16bit_diff_tick(cl.currentTime, cl.lastT);
        }
        cl.lastT = cl.currentTime;

        qAttitude.dt = ((float)cl.dtInUs / 1000.0f); // 4 ms
        
        // Get gyro Data
        MPU6050_data_t mpuData = MPU6050_getData();
        ERROR_CHECK(mpuData.errCode);
      
        // Calculate angles and quad-copter attitude.
        qdCalculateAttitude(&qAttitude, mpuData.gyro.x, mpuData.gyro.y, mpuData.gyro.z, mpuData.acc.x, mpuData.acc.y, mpuData.acc.z);
             
        // Calculate Setpoints from rc controller inputs
        receivers_t rc = rcGetChannels();
        qdCalculateSetPoints(&qAttitude, rc.pitch, rc.roll, rc.yaw, cl.gainLevelAdjust, cl.setPointDiv, true);

        // Get and filter raw gyro output and convert to degrees per second from rads. Using 65.5.
        quad_t pidInputs;
        pidInputs.pitch = filterComplementrary(pidInputs.pitch, mpuData.gyro.x, cl.gainCompFilter);
        pidInputs.roll  = filterComplementrary(pidInputs.roll, mpuData.gyro.y,  cl.gainCompFilter);
        pidInputs.yaw   = filterComplementrary(pidInputs.yaw, mpuData.gyro.z,   cl.gainCompFilter);

        // Calculate PID for each axis
        float rollOutput  = pidCalculate(&pidRoll,  pidInputs.roll,  qAttitude.rollSet,  cl.maxPidOutput);
        float pitchOutput = pidCalculate(&pidPitch, pidInputs.pitch, qAttitude.pitchSet, cl.maxPidOutput);
        float yawOutput   = pidCalculate(&pidYaw,   pidInputs.yaw,   qAttitude.yawSet,   cl.maxPidOutput);
                
        // Mix outputs to esc values
        esc_t esc = motorsMix(rc.throttle, pitchOutput, rollOutput, yawOutput, cl.minEscOutput, cl.maxEscOutput);
        
        // If enabled change motor pulse
        switch (rcSwitchGet3Way())
        {
            case RC_SWITCH_3WAY_OFF:
                // TODO: Reset PID here.
                pwm_update_duty_cycle_all(1000);
                pidReset(&pidPitch);
                pidReset(&pidRoll);
                pidReset(&pidYaw);
                break;
            case RC_SWITCH_3WAY_ON:
                pwm_update_duty_cycle(esc.esc3, esc.esc2, esc.esc1, esc.esc4); // Front Right - Front Left - Back Right - Back Left
                break;
            case RC_SWITCH_3WAY_ONPLUS:
                pwm_update_duty_cycle(esc.esc3, esc.esc2, esc.esc1, esc.esc4); // Front Right - Front Left - Back Right - Back Left
                break;
            default:
                LOG("ERROR: Switch 3 Undefined \r\n");
                pwm_update_duty_cycle_all(1000);
                break;
        }

        //nrf_delay_ms(2);
        if (!(cl.loopCounter % 160))
        {        
            float accPitch, accRoll;
        //  LOG("      RC: R:%5i,  P:%5i,  Y:%4i,  T:%5i \r\n", (int32_t)rc.roll, (int32_t) rc.pitch, (int32_t)rc.yaw, (int32_t)rc.throttle);
            LOG(" Acc Out: x:%5i,  y:%5i,  Z:%4i,        \r\n", (int32_t)mpuData.acc.x -98, (int32_t)mpuData.acc.y + 78, (int32_t)mpuData.acc.z + 20);
          LOG("   Angle: R:%5i,  P:%5i,  Y:%4i,        \r\n", (int32_t)qAttitude.roll, (int32_t)qAttitude.pitch, (int32_t)qAttitude.yaw);
        //  LOG(" Set Pnt: R:%5i,  P:%5i,  Y:%4i,        \r\n", (int32_t)qAttitude.rollSet, (int32_t)qAttitude.pitchSet, (int32_t)qAttitude.yawSet);
        //  LOG("  PID In: R:%5i,  P:%5i,  Y:%4i,        \r\n", (int32_t)pidInputs.roll, (int32_t)pidInputs.pitch, (int32_t)pidInputs.yaw);
        //  LOG(" PID Out: R:%5i,  P:%5i,  Y:%4i,        \r\n", (int32_t)rollOutput, (int32_t)pitchOutput, (int32_t)yawOutput);
        //  LOG("     ESC: 1:%5i,  2:%5i,  3:%4i,  4:%5i \r\n",(int32_t)esc.esc1, (int32_t)esc.esc2, (int32_t)esc.esc3, (int32_t)esc.esc4);
        //  LOG("      dT: T:%5i        \r\n", (int32_t)cl.dtInUs);
        }
        FLUSH();
        cl.loopCounter++;
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

//        uint16_t switch3WayVal = rc_get_3_way_switch();
//        if (switch3WayVal < 1900)
//        {
//            if (switch3WayVal < 1400)
//            {
//                pwm_update_duty_cycle(rc_get_throttle(), rc_get_throttle(), rc_get_throttle(), rc_get_throttle()); // Front Right - Front Left - Back Right - Back Left
//            }
//            else
//            {
//                pwm_update_duty_cycle(rc_get_throttle(), rc_get_throttle(), rc_get_throttle(), rc_get_throttle()); // Front Right - Front Left - Back Right - Back Left
//            }
//        }
//        else if (switch3WayVal > 1900)
//        { 
//            pwm_update_duty_cycle_all(1000);
//        }
//        
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
