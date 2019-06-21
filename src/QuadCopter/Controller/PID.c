#include "PID.h"

#include <stdint.h>
#include <math.h>



static float pidLimitOutput(float input, float max)
{
    float out = input;
    if (out > max)
    {
        out = max;
    }
    else if (out < (max * -1))
    {
        out = max * (-1);
    }
    return out;
}


float pidCalculate(pidConfig_t *pid, float input, float setPoint, float max)
{
  float errorP = setPoint - input;

  pid->errorI += pid->gains.i * errorP;
  pid->errorI = pidLimitOutput(pid->errorI, pid->maxI);

  float error_d = errorP - pid->errorPLast;
  float pid_output = (errorP * pid->gains.p) + pid->errorI + (pid->gains.d * error_d);
  pid_output = pidLimitOutput(pid_output, max);
  pid->errorPLast = errorP;

  return pid_output;
}


void pidReset(pidConfig_t *pid)
{
  pid->errorI = 0;
  pid->errorPLast = 0;
  pid->input = 0;
  pid->setPoint = 0;
}