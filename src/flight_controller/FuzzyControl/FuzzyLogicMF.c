#include "FuzzyLogicMF.h"

#include <stdint.h>
#include <stdbool.h>
#include "common_math.h"

#include "nrf_port.h"


//-----------------------------------------------------------------------------
// MF (Membership Functions)
//-----------------------------------------------------------------------------
float fuzzyMFGauss(float  x, float max, float min)
{
  float maxByTwo = (max / 2.0f);
  float centre = (max + min) / 2.0f;
  float top = x - centre;
  // TODO: Add min and max values as a input
  return exp((-1.0f) * ((top * top) / (maxByTwo * maxByTwo))); //center is 0
}


float fuzzyMFGaussInv(float x, float max, float min)
{
  return (1.0f - fuzzyMFGauss(x, max, min));
}


float fuzzyMFLinearPos(float val, float max, float min)
{
  if (val <= min) 
    val = min;
  else if (val >= max) 
    val = max;

  return ((val - min) / (max - min));
}
 

float fuzzyMFLinearNeg(float val, float max, float min)
{
  if (val <= min) 
    val = min;
  else if (val >= max) 
    val = max;

  return ((val - max) / (min - max));
}

