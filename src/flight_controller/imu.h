/**@file
 * 
 *
 *
 * @author Dan Lonngren
 */
#ifndef IMU_H_
#define IMU_H_

/**
 * TODO: Move Attitude estimation here from quaternion_math.c and quad_dynamics.c
*/
#include <stdint.h>
#include <stdbool.h>
#include "vector_math.h"
#include "quaternion_math.h"


#define twoKpDef    (2500.0f/10000.0f)	//!< 2 * proportional gain
#define twoKiDef    (50.0f / 10000.0f)	//!< 2 * integral gain


/**@brief Attitude Structure */
typedef struct {
    float roll, pitch, yaw;   //!< Euler's angles 
    vector3_t vRate;          //!< Rate Vector
    vector3_t vRateSP;        //!< Rate Vector Set point
} attitude_t;

typedef struct {
  float pitch;
  float roll;
  float yaw;
} attitudeEuler_t;

typedef struct {
  const float setPointLevelAdjust;
  const float setPointDiv;
  const float rollPitchCorrection;
  const float receiverCenter;
  bool autoLevel;
} imuConfig_t;

extern quaternion_t gImuCurrentOrientation;

extern attitude_t   gImuCurrentAttitude;

///////////////////////////////////////////////////////////////////////////////

void imuUpdateEulerAngles(attitude_t *attitude, const quaternion_t *q);


void imuMahonyAHRSupdate(quaternion_t *prevQ, vector3_t *vGyro, vector3_t *vAcc, vector3_t *magB, float dt);


void imuCalcSetPoints(attitude_t *att, vector3_t *rcAttitude, const imuConfig_t *cfg);


void imuCalcAngleFromAcc(float *pitch, float *roll, vector3_t * acc);


void imuCalcAngle(attitude_t *cAttitude, vector3_t *rcAttitude, vector3_t *acc, const imuConfig_t *cfg);

void imuUpdateAttitude(float dt);


#endif