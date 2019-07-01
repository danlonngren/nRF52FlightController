/*
 * quad_dynamics.c
 *
 * Magwick filter and fusion scheme
 *
 * https://github.com/kriswiner/GY-80/blob/master/GY80BasicAHRS.ino
 *
 * Created: 15/06/2019 17:30:59
 *  Author: Danlo
 */ 


#include "quaternion_math.h"

#include "common_math.h"

#include "vector_math.h"

#include "nrf_port.h"


#ifndef NULL
#define NULL  0
#endif

uint64_t counter = 0;
static float matx[3][3];

static quaternion_t currentOrientation;

typedef struct{
  float pitch, roll, yaw;
}attitudeEulers_t;

static attitudeEulers_t gAttitudeEuler;

static void imuUpdateEulerAngles(const quaternion_t *q, attitudeEulers_t *attitude)
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
        attitude->yaw += 360;

}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

#define twoKpDef      100.0f // (2500.0f/10000.0f)	// 2 * proportional gain
#define twoKiDef      50.0f // (50.0f / 10000.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)

vector3_t vCorrectedMagNorth = { .v = {0.0f,0.0f,1.0f}};

void MahonyAHRSupdateIMU(quaternion_t *prevQ, vector3_t *vGyro, vector3_t *vAcc, vector3_t *magB, float dt) 
{

        vector3_t vRotation = *vGyro;
        static vector3_t vGyroDriftEst = { 0 };

        const float genSpinRate = vector3NormSquare(&vRotation);


        static const vector3_t vForward = { .v = { 1.0f, 0.0f, 0.0f } };

        vector3_t vErr = { .v = { 1.0f, 0.0f, 0.0f } };

        if (magB && vector3NormSquare(magB) > 0.01f) {
            vector3_t vMag;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // This should yield direction to magnetic North (1; 0; 0)
            qMathRotateVectorInv(&vMag, magB, prevQ);    // BF -> EF

            // Ignore magnetic inclination
            //vMag.z = 0.0f;

            // We zeroed out vMag.z -  make sure the whole vector didn't go to zero
            if (vector3NormSquare(&vMag) > 0.01f) {
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
	if(vAcc) {
                
                static const vector3_t vGravity = { .v = { 0.0f, 0.0f, 1.0f}};
                vector3_t vEstGravity, vAccTemp, vErr;

                qMathRotateVector(&vEstGravity, &vGravity, prevQ); // BF

                vector3Normalize(&vAccTemp, vAcc);
              
                vector3CrossProduct(&vErr, &vAccTemp, &vEstGravity);

		if(twoKi > 0.0f) {
			if (genSpinRate < (toRadians(20)*toRadians(20)))
                        {
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

            if (thetaMagnitude < sqrtf(24.0f * 1e-6f)) {
                qMathScale(&deltaQ, &deltaQ, 1.0f - thetaMagnitude / 6.0f);
                deltaQ.q0 = 1.0f - thetaMagnitude / 2.0f;
            }
            else {
              const float thetaMag = sqrtf(thetaMagnitude);
              qMathScale(&deltaQ, &deltaQ, sin(thetaMag) / thetaMag);
              deltaQ.q0 = cos(thetaMag);
            }
        }

        qMathMultiply(prevQ, prevQ, &deltaQ);
        qMathNormalize(prevQ, prevQ);
}



quaternion_t qu = { .q0 = 1.0f, .q1 = 0, .q2 = 0, .q3 = 0};

void mQuaternionTest(vector3_t vGyro, vector3_t vAcc, vector3_t vMag, float dt)
{

    float yaw   = 0;
    float pitch = 0;
    float roll  = 0;

    vGyro.v[0] = toRadians(vGyro.v[0]);
    vGyro.v[1] = toRadians(vGyro.v[1]);
    vGyro.v[2] = toRadians(vGyro.v[2]);

    MahonyAHRSupdateIMU( &qu, &vGyro, &vAcc, &vMag, dt);

    imuUpdateEulerAngles(&qu, &gAttitudeEuler);
  
  if (!(counter % 50))
  {

    LOG("q Angles: P:%4i, R:%4i, Y:%4i \r\n", (int32_t)pitch, (int32_t)roll, (int32_t)yaw);
    
    FLUSH();
  }
  counter++;
}