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

uint64_t counter= 0;


#define betaDef		3.0f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)

float qRate[4]; 


void MadgwickQuaternionUpdate(float *q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;

    hx =      mx * q1q1 - 
            _2q1my * q4 + 
            _2q1mz * q3 + 
              mx * q2q2 + 
         _2q2 * my * q3 + 
         _2q2 * mz * q4 - 
              mx * q3q3 - 
              mx * q4q4;
    
    hy =    _2q1mx * q4 + 
              my * q1q1 - 
            _2q1mz * q2 + 
            _2q2mx * q3 - 
              my * q2q2 + 
              my * q3q3 + 
         _2q3 * mz * q4 - 
               my * q4q4;
    
    _2bx = sqrt(hx * hx + hy * hy);

    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) +
          _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - 
          _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + 
          _2bz * (q2q4 - q1q3) - mx) + 
          (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + 
          _2bz * (q1q2 + q3q4) - my) + 
          _2bx * q3 * (_2bx * (q1q3 + q2q4) + 
          _2bz * (0.5f - q2q2 - q3q3) - mz);
    
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + 
         _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 
         4.0f * q2 * (1.0f - 2.0f * q2q2 - 
         2.0f * q3q3 - az) + 
         _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + 
         _2bz * (q2q4 - q1q3) - mx) + 
         (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + 
         _2bz * (q1q2 + q3q4) - my) + 
         (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + 
         _2bz * (0.5f - q2q2 - q3q3) - mz);
    
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + 
          _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 
          4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + 
          (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + 
          _2bz * (q2q4 - q1q3) - mx) + 
          (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + 
          _2bz * (q1q2 + q3q4) - my) + 
          (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + 
          _2bz * (0.5f - q2q2 - q3q3) - mz);
    
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
      if (!(counter % 50))
  {
   // LOG("qDot: %i, %i, %i, %i \r\n", (int32_t)(qDot1*1000.0f), (int32_t)(qDot2*1000.0f), (int32_t)(qDot3*1000.0f), (int32_t)(qDot4*1000.0f));
   
    }// Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;

    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.1f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

volatile float q0, q1, q2, q3;	

void MahonyAHRSupdateIMU(float *qu, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
        float q0, q1, q2, q3;
        q0 = qu[0];
        q1 = qu[1];
        q2 = qu[2];
        q3 = qu[3];
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

        qu[0] = q0;
        qu[1] = q1;
        qu[2] = q2;
        qu[3] = q3;
}



 float qu[4] = {1.0f, 0, 0, 0};

void mQuaternionTest(axis_t data, float dt)
{

    float yaw   = 0;
    float pitch = 0;
    float roll  = 0;


//  MadgwickAHRSupdate
  //MahonyAHRSupdateIMU( qu, data.gyro.x,  data.gyro.y,  data.gyro.z, data.acc.x,   data.acc.y,   data.acc.z,dt);

  MadgwickQuaternionUpdate( qu, data.acc.x,   data.acc.y,   data.acc.z, 
                          data.gyro.x,  data.gyro.y,  data.gyro.z,
                          data.mag.x,   data.mag.y,   data.mag.z, 
                                   dt); // dt in seconds

  quaternion_t quatern;
  qMathSetQ(&quatern, qu);

  qMathToEuler(&quatern, &pitch, &roll, &yaw);

  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  roll  *= 180.0f / PI;
  
  
  if (!(counter % 50))
  {
    float qTest1[4] = {1,0,0,0};
    float qTest2[4] = {1,0,0,0};
    qMathAnglesToQ(qTest1, 0, 0.0f, 50.0f* PI/180.0f);
    
    
    float prod[4] = {1.0f,0.0f,0.0f};

    qMathProductQ(prod, qu, qTest1);
    
    LOG("q       : q1:%4i, q2:%4i, q3:%4i - \r\n", (int32_t)(qu[1]*1000.0f), (int32_t)(qu[2]*1000.0f), (int16_t)(qu[3]*1000.0f));
    LOG("qTest   : q1:%4i, q2:%4i, q3:%4i - ",  (int32_t)(prod[1]*1000.0f), (int32_t)(prod[2]*1000.0f), (int16_t)(prod[3]*1000.0f));
    LOG(" Test: %i \r\n", dt);
    LOG("q Angles: P:%4i, R:%4i, Y:%4i \r\n", (int32_t)pitch, (int32_t)roll, (int32_t)yaw);
    
    FLUSH();
  }
  counter++;
}