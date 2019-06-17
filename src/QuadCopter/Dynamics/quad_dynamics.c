/*
 * quad_dynamics.c
 *
 * Created: 15/06/2019 17:30:59
 *  Author: Danlo
 */ 


#include "quad_dynamics.h"
#include <math.h>

#ifndef PI
#define PI  3.14159
#endif

/* 
@TODO:
    Add Startup Sequence setting the correct angle using the 	
    Add filtering
    
    Calculate setpoint

*/

float filterAngle;
float dt=0.02;

float comp_filter(float newAngle, float newRate) {

    float filterTerm0;
    float filterTerm1;
    float filterTerm2;
    float timeConstant;
    
    timeConstant=0.5; // default 1.0
    
    filterTerm0 = (newAngle - filterAngle) * timeConstant * timeConstant;
    filterTerm2 += filterTerm0 * dt;
    filterTerm1 = filterTerm2 + ((newAngle - filterAngle) * 2 * timeConstant) + newRate;
    filterAngle = (filterTerm1 * dt) + filterAngle;
    float previousAngle = filterAngle;
    return previousAngle; // This is actually the current angle, but is stored for the next iteration
}


void qdCalcAccAngles(float *aX, float *aY, float x, float y, float z)
{
    uint32_t acc_vector = sqrt((x * x) + (y * y) + (z * z));
    *aX = asin((float)y / (float)acc_vector) * 57.296;
    *aY = asin((float)x / (float)acc_vector) * -57.296;
}

void qdCalculateAngleFromAcc(float *pitch, float *roll, int16_t ax, int16_t ay, int16_t az)
{
    float accPitch = 0, accRoll = 0;

    float acc_vector = sqrtf(((float)ax * ax) + ((float)ay * ay) + ((float)az * az));
    if (fabs(ay) < acc_vector)
    {
    	accPitch = asin((float)ay / (float)acc_vector) * 57.296;
    }
    if (fabs(ax) < acc_vector)
    {	
        accRoll = asin((float)ax / (float)acc_vector) * -57.296;
    }
    *pitch = accPitch;
    *roll = accRoll;
}

void qdCalculateAttitude(attitude_t *cAttitude, int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az)
{
    float constant1 = cAttitude->dt / 1000.0 / 65.5;
    cAttitude->pitch += (float)gx * constant1;
    cAttitude->roll  += (float)gy * constant1;
    cAttitude->yaw    += (float)gz * constant1;
    
    float constant2 = sin((float)gz * (constant1 * 0.017453277)); // (PI / 180.0)
    cAttitude->pitch += cAttitude->roll * constant2;
    cAttitude->roll  -= cAttitude->pitch * constant2;
    
    float accPitch = 0, accRoll = 0;
    qdCalculateAngleFromAcc(&accPitch, &accRoll, ax, ay, az);
    // Fuse acc And Gyro data
    cAttitude->pitch = cAttitude->pitch * 0.9996 + accPitch * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    cAttitude->roll = cAttitude->roll * 0.9996 + accRoll * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

}

