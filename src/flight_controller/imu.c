/*
 * quad_dynamics.c
 *
 * Created: 15/06/2019 17:30:59
 *  Author: Danlo
 */ 


#include "imu.h"

#include "parameters.h"

#include "filters.h"
#include "nrf_port.h"

#include "receivers.h"
#include "imu_sensors.h"

static float matx[3][3];

quaternion_t gImuCurrentOrientation;
attitude_t   gImuCurrentAttitude;

/* Raw IMU Data in Rads, g, mG */
static vector3_t gImuGyroRateRads;
static vector3_t gImuAccG;
static vector3_t gImuMagMG;

imuConfig_t imuConfig = {
    .setPointLevelAdjust  = RECEIVER_LEVEL_ADJUST_GAIN,
    .setPointDiv          = SET_POINT_DIV,
    .rollPitchCorrection  = 0.9998f,
    .autoLevel            = true,
};


imuConfig_t * imuConfigGet(void)
{
    return &imuConfig;
}


void imuUpdateEulerAngles(attitude_t *attitude, const quaternion_t *q)
{
       
    float q1q1 = q->q1 * q->q1;
    float q2q2 = q->q2 * q->q2;
    float q3q3 = q->q3 * q->q3;     
    float q0q1 = q->q0 * q->q1;
    float q0q2 = q->q0 * q->q2;
    float q0q3 = q->q0 * q->q3;
    float q1q2 = q->q1 * q->q2;
    float q1q3 = q->q1 * q->q3;
    float q2q3 = q->q2 * q->q3;

    matx[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    matx[0][1] = 2.0f * (q1q2 + -q0q3);
    matx[0][2] = 2.0f * (q1q3 - -q0q2);

    matx[1][0] = 2.0f * (q1q2 - -q0q3);
    matx[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    matx[1][2] = 2.0f * (q2q3 + -q0q1);

    matx[2][0] = 2.0f * (q1q3 + -q0q2);
    matx[2][1] = 2.0f * (q2q3 - -q0q1);
    matx[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;

    /* Compute pitch/roll angles */
    attitude->roll = toDegrees(atan2_approx(matx[2][1], matx[2][2]));
    attitude->pitch = toDegrees((0.5f * PI) - acos_approx(-matx[2][0]));
    attitude->yaw = toDegrees(-atan2_approx(matx[1][0], matx[0][0]));

    if (attitude->yaw < 0)
    {
        attitude->yaw += 360.0f;
    }
}



//-----------------------------------------------------------------------------
// Variable definitions
//-----------------------------------------------------------------------------
volatile float twoKp = twoKpDef;  // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;  // 2 * integral gain (Ki)
vector3_t vCorrectedMagNorth = { .v = {0.0f, 0.0f, 1.0f} }; //!< Magnetic north variable (Not corrected)

void imuMahonyAHRSupdate(quaternion_t *prevQ, vector3_t *vGyro, vector3_t *vAcc, vector3_t *magB, float dt) 
{
    vector3_t vRotation = *vGyro;
    static vector3_t vGyroDriftEst = { 0 };
    static const vector3_t vForward = { .v = { 1.0f, 0.0f, 0.0f } };
    const float genSpinRate = vector3NormSquare(&vRotation);
    vector3_t vErr = { .v = { 1.0f, 0.0f, 0.0f } };
    
    if (magB && vector3NormSquare(magB) > 0.01f) 
    {
        vector3_t vMag;
    
        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
    
        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // This should yield direction to magnetic North (1; 0; 0)
        qMathRotateVectorInv(&vMag, magB, prevQ);    // BF -> EF
    
        // Ignore magnetic inclination
        //vMag.z = 0.0f;
    
        // We zeroed out vMag.z -  make sure the whole vector didn't go to zero
        if (vector3NormSquare(&vMag) > 0.01f) 
        {
            // Normalize to unit vector
            vector3Normalize(&vMag, &vMag);
    
            // Reference mag field vector heading is Magnetic North in EF. We compute that by rotating True North vector by declination and assuming Z-component is zero
            // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
            vector3CrossProduct(&vErr, &vMag, &vCorrectedMagNorth);
    
            // Rotate error back into body frame
            qMathRotateVector(&vErr, &vErr, prevQ);
        }
    }
    
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(vAcc) 
    {
            
        static const vector3_t vGravity = { .v = { 0.0f, 0.0f, 1.0f}};
        vector3_t vEstGravity, vAccTemp, vErr;
    
        qMathRotateVector(&vEstGravity, &vGravity, prevQ); // BF
    
        vector3Normalize(&vAccTemp, vAcc);
        
        vector3CrossProduct(&vErr, &vAccTemp, &vEstGravity);
    
        if(twoKi > 0.0f) 
        {
            if (genSpinRate < (toRadians(20.0f)*toRadians(20.0f))){
    
                vector3_t vTemp;
                
                vector3Scale(&vTemp, &vErr, twoKi * dt);
                vector3Add(&vGyroDriftEst, &vGyroDriftEst, &vTemp); // Gyro drift estimate
    
            }     
        }
    
        vector3Scale(&vErr, &vErr, twoKp);
        vector3Add(&vRotation, &vRotation, &vErr);
    }
    
    vector3Add(&vRotation, &vRotation, &vGyroDriftEst);
    
    
    vector3_t vTheta;
    quaternion_t deltaQ;
    
    vector3Scale(&vTheta, &vRotation, 0.5f * dt);
    qMathInitVector(&deltaQ, &vTheta);
    
    const float thetaMagnitude = vector3NormSquare(&vTheta);
    
    if (thetaMagnitude >= 1e-20)
    {
    
        if (thetaMagnitude < sqrtf(24.0f * 1e-6f)) 
        {
            qMathScale(&deltaQ, &deltaQ, 1.0f - thetaMagnitude / 6.0f);
            deltaQ.q0 = 1.0f - thetaMagnitude / 2.0f;
        }
        else 
        {
            const float thetaMag = sqrtf(thetaMagnitude);
            qMathScale(&deltaQ, &deltaQ, sin(thetaMag) / thetaMag);
            deltaQ.q0 = cos(thetaMag);
        }
    }
    
    qMathMultiply(prevQ, prevQ, &deltaQ);
    qMathNormalize(prevQ, prevQ);
}


void imuCalcSetPoints(attitude_t *att, vector3_t *rcAttitude, const imuConfig_t *cfg)
{
    float rollAdjust  = att->roll * cfg->setPointLevelAdjust;
    float pitchAdjust = att->pitch * cfg->setPointLevelAdjust;
    float setPointDiv = cfg->setPointDiv;

    // If auto-level enable add pitch adjust.
    if (cfg->autoLevel == false)
    {
        pitchAdjust = 0.0f;
        rollAdjust = 0.0f;
    }

    // Calculate set points for PID.
    vector3_t tempRateSP;
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

    att->vRateSP.x = filterSinglePoleLowPass(&att->vRateSP.x, &tempRateSP.x, 0.015f); //!< Dampen for smoother flight
    att->vRateSP.y = filterSinglePoleLowPass(&att->vRateSP.y, &tempRateSP.y, 0.015f); //!< Dampen for smoother flight
    att->vRateSP.z = filterSinglePoleLowPass(&att->vRateSP.z, &tempRateSP.z, 0.015f); //!< Dampen for smoother flight
}

/**@breif Calculate angles from accelerometer */
void imuCalcAngleFromAcc(float *pitch, float *roll, const vector3_t * acc)
{
    float accPitch = 0.0f; 
    float accRoll = 0.0f;

    float acc_vector = sqrtf(vector3NormSquare(acc));

    if (fabs(acc->y) < acc_vector)
    {
    	accRoll = asin(acc->y / acc_vector) * 57.296f;
    }
    if (fabs(acc->x) < acc_vector)
    {	
        accPitch = asin(acc->x / acc_vector) * -57.296f;
    }
    *pitch = accPitch;
    *roll  = accRoll;
}


void imuUpdateSensors(int32_t param)
{
    // Get sensor data
    // TODO: Improve this by changing to interrupt base
    imuSensorGetData(&gImuGyroRateRads, &gImuAccG, &gImuMagMG);

    gImuCurrentAttitude.vRate = gImuGyroRateRads;
    
    // Scale mag value down to uG
    vector3Scale(&gImuMagMG, &gImuMagMG, 0.001f);// 1.0f/1000.0f);
    // Manual offset
    gImuMagMG.v[2] -= 0.10f; 
        
}

void imuUpdateAttitude(float dt)
{
    // Get receiver channels and add to vector
    // TODO: Improve this by converting to quaternions and get difference from current attitude.
    receivers_t rc = rcGetChannels();
    vector3_t rcAttitude = { .v = {rc.roll, rc.pitch, rc.yaw }};

    // Convert rate to radians and calculate attitude.
    vector3_t vRateRads;
    vector3Scale(&vRateRads, &gImuCurrentAttitude.vRate, TO_RADIANS);

    /* Update Orientation (Quaternion)*/
    imuMahonyAHRSupdate(&gImuCurrentOrientation, &vRateRads, &gImuAccG, &gImuMagMG, dt);
    
    /* Calculate Attitude in terms of Euler's angles */
    imuUpdateEulerAngles(&gImuCurrentAttitude, &gImuCurrentOrientation);
    
    /* Calculate setpoints */
    imuCalcSetPoints(&gImuCurrentAttitude, &rcAttitude, &imuConfig);
}



#if 0
void imuCalcAngle(attitude_t *cAttitude, vector3_t *rcAttitude, vector3_t *acc, const imuConfig_t *cfg)
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
    imuCalcAngleFromAcc(&accPitch, &accRoll, acc);

    // Fuse acc And Gyro data
    vAngle->y = FILTER_COMP(vAngle->y, accPitch, cfg->rollPitchCorrection);
    vAngle->x = FILTER_COMP(vAngle->x, accRoll,  cfg->rollPitchCorrection);

    qdCalculateSetPoints(cAttitude, rcAttitude, cfg);
}
#endif

