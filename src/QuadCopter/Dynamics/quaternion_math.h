

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

#include "common_math.h"

#include <stdint.h>
#include <stdbool.h>

#include "imu_sensors.h"


typedef struct
{
  float q[4];
} quaternion_t;

typedef struct
{
    float x;
    float y;
    float z;	
}xyz_t;

typedef struct  
{
    xyz_t gyro;
    xyz_t acc;
    xyz_t mag;
    error_t errCode;
}axis_t;

void mQuaternionTest(axis_t data, float dt);

/*------------------------------Inline Math---------------------------------*/
static inline void qMathInitialiseQ(quaternion_t *q, float *qIn)
{
  q->q[0] = 1.0f;
  q->q[1] = 0.0f;
  q->q[2] = 0.0f;
  q->q[3] = 0.0f;
}


static inline void qMathSetQ(quaternion_t *q, float *qIn)
{
  q->q[0] = qIn[0];
  q->q[1] = qIn[1];
  q->q[2] = qIn[2];
  q->q[3] = qIn[3];
}

static inline void qMathToEuler(quaternion_t *qu, float *pitch, float *roll, float *yaw)
{
  float q0q0 = qu->q[0] * qu->q[0];  float q1q1 = qu->q[1] * qu->q[1];
  float q2q2 = qu->q[2] * qu->q[2];  float q3q3 = qu->q[3] * qu->q[3];
  float t1 = qu->q[1] * qu->q[2] + qu->q[0] * qu->q[3];
  float t2 = (qu->q[1] * qu->q[3] - qu->q[0] * qu->q[2]);
  float t3 = (qu->q[0] * qu->q[1] + qu->q[2] * qu->q[3]);

  *yaw   = atan2(2.0f * t1, q0q0 + q1q1 - q2q2 - q3q3);   
  *pitch = -asin(2.0f * t2);
  *roll  = atan2(2.0f * t3, q0q0 - q1q1 - q2q2 + q3q3);
}


static inline void qMathProductQ(float *prod, float *q1, float *q2)
{
  
   prod[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; 
   prod[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];                          
   prod[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
   prod[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];

}


static inline void qMathAnglesToQ(float *q, double pitch, double roll, double yaw)
{
	float t0 = cos(yaw * 0.5f);
	float t1 = sin(yaw * 0.5f);
	float t2 = cos(roll * 0.5f);
	float t3 = sin(roll * 0.5f);
	float t4 = cos(pitch * 0.5f);
	float t5 = sin(pitch * 0.5f);

	q[0] = t0 * t2 * t4 + t1 * t3 * t5;
	q[1] = t0 * t3 * t4 - t1 * t2 * t5;
	q[2] = t0 * t2 * t5 + t1 * t3 * t4;
	q[3] = t1 * t2 * t4 - t0 * t3 * t5;
	// Normalise quaternion
	float recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;

}


#endif