

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

#include "common_math.h"
#include "vector_math.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  float q0, q1, q2, q3;
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
}axis_t;


/*------------------------------Inline Math---------------------------------*/
static inline void qMathInitialiseQ(quaternion_t *q, float *qIn)
{
  q->q0 = 1.0f;
  q->q1 = 0.0f;
  q->q2 = 0.0f;
  q->q3 = 0.0f;
}

static inline quaternion_t * qMathInitVector(quaternion_t * result, const vector3_t * v)
{
    result->q0 = 0.0f;
    result->q1 = v->x;
    result->q2 = v->y;
    result->q3 = v->z;
    return result;
}
//
//static inline void qMathSetQ(quaternion_t *q, float *qIn)
//{
//  q->q0 = qIn0;
//  q->q1 = qIn1;
//  q->q2 = qIn2;
//  q->q3 = qIn3;
//}

static inline void qMathToEuler(const quaternion_t *qu, float *pitch, float *roll, float *yaw)
{
  float q0q0 = qu->q0 * qu->q0;  
  float q1q1 = qu->q1 * qu->q1;
  float q2q2 = qu->q2 * qu->q2;  
  float q3q3 = qu->q3 * qu->q3;
  float t1 =   qu->q1 * qu->q2 + qu->q0 * qu->q3;
  float t2 =  (qu->q1 * qu->q3 - qu->q0 * qu->q2);
  float t3 =  (qu->q0 * qu->q1 + qu->q2 * qu->q3);

  *yaw   = atan2(2.0f * t1, q0q0 + q1q1 - q2q2 - q3q3);   
  *pitch = -asin(2.0f * t2);
  *roll  = atan2(2.0f * t3, q0q0 - q1q1 - q2q2 + q3q3);
}


static inline float qMathNormSqared(const quaternion_t * q)
{
    return (q->q0*q->q0) + (q->q1*q->q1) + (q->q2*q->q2) + (q->q3*q->q3);
}


static inline quaternion_t * qMathScale(quaternion_t * result, const quaternion_t * a, const float b)
{
    quaternion_t p;

    p.q0 = a->q0 * b;
    p.q1 = a->q1 * b;
    p.q2 = a->q2 * b;
    p.q3 = a->q3 * b;

    *result = p;
    return result;
}

static inline quaternion_t * qMathMultiply(quaternion_t *result, const quaternion_t *a, const quaternion_t *b)
{
  quaternion_t p;
  p.q0 = a->q0 * b->q0 - a->q1 * b->q1 - a->q2 * b->q2 - a->q3 * b->q3;
  p.q1 = a->q0 * b->q1 + a->q1 * b->q0 + a->q2 * b->q3 - a->q3 * b->q2;
  p.q2 = a->q0 * b->q2 - a->q1 * b->q3 + a->q2 * b->q0 + a->q3 * b->q1;
  p.q3 = a->q0 * b->q3 + a->q1 * b->q2 - a->q2 * b->q1 + a->q3 * b->q0;
  *result = p;
  return result;
}

static inline quaternion_t * qMathConjugate(quaternion_t *result, const quaternion_t * q)
{
    result->q0 =  q->q0;
    result->q1 = -q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;

    return result;
}

static inline vector3_t * qMathRotateVector(vector3_t * result, const vector3_t * vect, const quaternion_t * ref)
{
    quaternion_t vectQuat, refConj;

    vectQuat.q0 = 0;
    vectQuat.q1 = vect->x;
    vectQuat.q2 = vect->y;
    vectQuat.q3 = vect->z;

    qMathConjugate(&refConj, ref);
    qMathMultiply(&vectQuat, &refConj, &vectQuat);
    qMathMultiply(&vectQuat, &vectQuat, ref);

    result->x = vectQuat.q1;
    result->y = vectQuat.q2;
    result->z = vectQuat.q3;
    return result;
}


static inline vector3_t * qMathRotateVectorInv(vector3_t * result, const vector3_t * vect, const quaternion_t * ref)
{
    quaternion_t vectQuat, refConj;

    vectQuat.q0 = 0;
    vectQuat.q1 = vect->x;
    vectQuat.q2 = vect->y;
    vectQuat.q3 = vect->z;

    qMathConjugate(&refConj, ref);
    qMathMultiply(&vectQuat, ref, &vectQuat);
    qMathMultiply(&vectQuat, &vectQuat, &refConj);

    result->x = vectQuat.q1;
    result->y = vectQuat.q2;
    result->z = vectQuat.q3;
    return result;
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


static inline quaternion_t * qMathNormalize(quaternion_t * result, const quaternion_t * q)
{
    float mod = sqrtf(qMathNormSqared(q));
    if (mod < 1e-6f) {
        // Length is too small - re-initialize to zero rotation
        result->q0 = 1;
        result->q1 = 0;
        result->q2 = 0;
        result->q3 = 0;
    }
    else {
        result->q0 = q->q0 / mod;
        result->q1 = q->q1 / mod;
        result->q2 = q->q2 / mod;
        result->q3 = q->q3 / mod;
    }

    return result;
}

#endif