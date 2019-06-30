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

typedef struct  
{
    float dt;
    float pitch;
    float roll;
    float yaw;
    float pitchSet;
    float rollSet;
    float yawSet;
} attitude_t;

typedef struct
{
  float pitch;
  float roll;
  float yaw;
} quad_t;


void qdCalculateAttitudeTest(attitude_t *outAttitude, attitude_t *gyro, attitude_t *acc, uint32_t dt);


void qdCalculateAngleFromAcc(float *pitch, float *roll, float ax, float ay, float az);


void qdCalculateAngle(attitude_t *cAttitude, float gyroRate[], float acc[]);


void qdCalculateSetPoints(attitude_t *att, float rcPitch, float rcRoll, float rcYaw, float adjustGain, float setPointDiv, bool autoLevel);


#endif /* QUAD_DYNAMICS_H_ */