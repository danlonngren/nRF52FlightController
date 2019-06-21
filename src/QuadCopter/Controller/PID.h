#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct
{
  float p;
  float i;
  float d;
} pid_gains_t;

typedef struct
{
  float input;
  float setPoint;
  float errorI;
  float errorPLast;
  const float maxI;
  const pid_gains_t gains;
} pidConfig_t;


float pidCalculate(pidConfig_t *pid, float input, float setPoint, float max);

void pidReset(pidConfig_t *pid);


#endif