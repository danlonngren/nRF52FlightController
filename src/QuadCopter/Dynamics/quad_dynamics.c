/*
 * quad_dynamics.c
 *
 * Created: 15/06/2019 17:30:59
 *  Author: Danlo
 */ 


#include "quad_dynamics.h"

#include "filters.h"

#include "nrf_port.h"

#include "common_math.h"


/* 
@TODO:
    Add Startup Sequence setting the correct angle using the 	
    Add filtering
    
    Calculate setpoint

*/


/**@breif Calculate angles from accelerometer */
void qdCalculateAngleFromAcc(float *pitch, float *roll, float ax, float ay, float az)
{
    float accPitch = 0, accRoll = 0;

    float acc_vector = sqrtf(((float)ax * ax) + ((float)ay * ay) + ((float)az * az));
    if (fabs(ay) < acc_vector)
    {
    	accRoll = asin((float)ay / (float)acc_vector) * 57.296;
    }
    if (fabs(ax) < acc_vector)
    {	
        accPitch = asin((float)ax / (float)acc_vector) * -57.296;
    }
    *pitch = accPitch;
    *roll  = accRoll;
}

void qdCalculateAngle(attitude_t *cAttitude, float gyroRate[], float acc[])
{
    float constant1 = cAttitude->dt; // Convert into seconds
    cAttitude->roll += gyroRate[0] * constant1;
    cAttitude->pitch  += gyroRate[1] * constant1;
    cAttitude->yaw = gyroRate[2]; // += (float)gz * constant1;
    
    float constant2 = sin(gyroRate[2] * (constant1 * 0.017453277)); // (PI / 180.0)
    cAttitude->pitch += cAttitude->roll * constant2;
    cAttitude->roll  -= cAttitude->pitch * constant2;
    
    float accPitch = 0, accRoll = 0;
    qdCalculateAngleFromAcc(&accPitch, &accRoll, acc[0], acc[1], acc[2]);

    // Fuse acc And Gyro data
    cAttitude->pitch = FILTER_COMP(cAttitude->pitch, accPitch, 0.9998);
    cAttitude->roll  = FILTER_COMP(cAttitude->roll, accRoll,   0.9998);
}


void qdCalculateSetPoints(attitude_t *att, float rcPitch, float rcRoll, float rcYaw, float adjustGain, float setPointDiv, bool autoLevel)
{
    float pitchAdjust = att->pitch * adjustGain;
    float rollAdjust  = att->roll * adjustGain;

    // If auto-level enable add pitch adjust.
    if (autoLevel == false)
    {
        pitchAdjust = 0;
        rollAdjust = 0;
    }

    // Get receivers and calculate setPoints for PID.
    att->pitchSet  = (rcPitch - 1500.0f);
    att->pitchSet -= pitchAdjust;
    att->pitchSet /= setPointDiv;

    att->rollSet   = (rcRoll - 1500.0f);
    att->rollSet  -= rollAdjust;
    att->rollSet  /= setPointDiv;

    att->yawSet    = (1500.0f - rcYaw);
    att->yawSet   /= setPointDiv;

}
