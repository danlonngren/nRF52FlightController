/*
 * quad_dynamics.h
 *
 * Created: 15/06/2019 17:31:17
 *  Author: Danlo
 */ 


#ifndef QUAD_DYNAMICS_H_
#define QUAD_DYNAMICS_H_

#include <stdint.h>
#include <stdbool.h>
#include "vector_math.h"


typedef struct  
{
    float dt;
    vector3_t angleRPY;
    vector3_t vRate;
    vector3_t vRateSP;
} attitude_t;

typedef struct
{
  float pitch;
  float roll;
  float yaw;
} quad_t;

typedef struct {
  const float setPointLevelAdjust;
  const float setPointDiv;
  const float rollPitchCorrection;
  const float receiverCenter;
  bool autoLevel;
} qd_config_t;


void qdCalculateAttitudeTest(attitude_t *outAttitude, attitude_t *gyro, attitude_t *acc, uint32_t dt);


void qdCalculateAngleFromAcc(float *pitch, float *roll, vector3_t * acc);


void qdCalculateAngle(attitude_t *cAttitude, vector3_t *rcAttitude, vector3_t *acc, const qd_config_t *cfg);



#endif /* QUAD_DYNAMICS_H_ */