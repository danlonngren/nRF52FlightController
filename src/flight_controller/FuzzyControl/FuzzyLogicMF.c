#include "FuzzyLogicMF.h"

#include <stdint.h>
#include <stdbool.h>
#include "common_math.h"

#include "nrf_port.h"



//-----------------------------------------------------------------------------
// MF (Membership Functions)
//-----------------------------------------------------------------------------
float funcGauss(float  x, float a)
{
  float y = 0;
  // TODO: Add min and max values as a input
  y = exp((-1.0f * (x * x)) / (2.0f * a)); //center is 0
  return y;
}


float linearMFPos(float val, float max, float min)
{
  if (val <= min) 
    val = min;
  else if (val >= max) 
    val = max;

  return ((val - min) / (max - min));
}
 
float linearMFNeg(float val, float max, float min)
{
  if (val <= min) 
    val = min;
  else if (val >= max) 
    val = max;

  return ((val - max) / (min - max));
}


int funcConvertStick(int in) 
{
  int in2 = in - 1500;
  int out = 0;

  if (in > 1500)
     out = ((in2 * in2) / 1000 + (in2) / 2 + 1500);
  else if (in < 1500)
     out =  (1500 - (in2 * in2) / 1000 + in2 / 2);
 return out;
}

int funcConvertThrottle(int in) {
  int Throttle = sqrt(in - 1000) * 31.63 + 1000;
  return Throttle;
}