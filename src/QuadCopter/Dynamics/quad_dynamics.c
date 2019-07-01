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
void qdCalculateAngleFromAcc(float *pitch, float *roll, vector3_t * acc)
{
    float accPitch = 0, accRoll = 0;

    float acc_vector = sqrtf(vector3NormSquare(acc));

    if (fabs(acc->y) < acc_vector)
    {
    	accRoll = asin(acc->y / acc_vector) * 57.296;
    }
    if (fabs(acc->x) < acc_vector)
    {	
        accPitch = asin(acc->x / acc_vector) * -57.296;
    }
    *pitch = accPitch;
    *roll  = accRoll;
}



static void qdCalculateSetPoints(attitude_t *att, vector3_t *rcAttitude, const qd_config_t *cfg)
{
    float rollAdjust  = att->angleRPY.x * cfg->setPointLevelAdjust;
    float pitchAdjust = att->angleRPY.y * cfg->setPointLevelAdjust;
    float setPointDiv = cfg->setPointDiv;
    // If auto-level enable add pitch adjust.
    if (cfg->autoLevel == false)
    {
        pitchAdjust = 0;
        rollAdjust = 0;
    }

    vector3_t tempRateSP;
    // Get receivers and calculate setPoints for PID.
    // Roll
    tempRateSP.x   = (rcAttitude->x - 1500.0f);
    tempRateSP.x  -= rollAdjust;
    tempRateSP.x  /= setPointDiv;
    // Pitch
    tempRateSP.y  = (rcAttitude->y - 1500.0f);
    tempRateSP.y -= pitchAdjust;
    tempRateSP.y /= setPointDiv;
    // Yaw
    tempRateSP.z    = (1500.0f - rcAttitude->z);
    tempRateSP.z   /= setPointDiv;

    att->vRateSP.x = FILTER_COMP(tempRateSP.x, att->vRateSP.x, 0.2f);
    att->vRateSP.y = FILTER_COMP(tempRateSP.y, att->vRateSP.y, 0.2f);
    att->vRateSP.z = FILTER_COMP(tempRateSP.z, att->vRateSP.z, 0.2f);
}


void qdCalculateAngle(attitude_t *cAttitude, vector3_t *rcAttitude, vector3_t *acc, const qd_config_t *cfg)
{
    vector3_t *gyroRate = &cAttitude->vRate;
    vector3_t *vAngle = &cAttitude->angleRPY;

    float constant1 = cAttitude->dt; // Convert into seconds
    vAngle->x  += gyroRate->v[0] * constant1;
    vAngle->y  += gyroRate->v[1] * constant1;
    vAngle->z   = gyroRate->v[2]; 
    
    float constant2 = sin(gyroRate->v[2] * (toRadians(constant1))); // (PI / 180.0)
    vAngle->y  += vAngle->x * constant2;
    vAngle->x  -= vAngle->y * constant2;
    
    float accPitch = 0, accRoll = 0;
    qdCalculateAngleFromAcc(&accPitch, &accRoll, acc);

    // Fuse acc And Gyro data
    vAngle->y = FILTER_COMP(vAngle->y, accPitch, cfg->rollPitchCorrection);
    vAngle->x = FILTER_COMP(vAngle->x, accRoll,  cfg->rollPitchCorrection);

    qdCalculateSetPoints(cAttitude, rcAttitude, cfg);
}

