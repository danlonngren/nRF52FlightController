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
#include "common_math.h"
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
  float setPointLevelAdjust;
  float setPointDiv;
  const float rollPitchCorrection;
  const float receiverCenter;
  bool autoLevel;
} imuConfig_t;

///////////////////////////////////////////////////////////////////////////////

imuConfig_t * imu_fusion_ConfigGet(void);

void imu_fusion_UpdateSensors(int32_t param);

void imu_fusion_UpdateAttitude(float dt);

void imu_fusion_CalcAngleFromAcc(float *pitch, float *roll, const vector3_t * acc);

quaternion_t *imu_fusion_GetCurrentOrientation( void );

attitude_t *imu_fusion_GetCurrentAttitude( void );

#endif