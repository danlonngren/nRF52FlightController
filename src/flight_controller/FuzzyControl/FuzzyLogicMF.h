#ifndef FUZZYLOGICMF_H
#define FUZZYLOGICMF_H

#include <stdint.h>

float funcGauss(float  x, float a);

float fuzzyMFLinearPos(float val, float max, float min);

float fuzzyMFLinearNeg(float val, float max, float min);

float fuzzyMFGauss(float  x, float max, float min);

float fuzzyMFGaussInv(float x, float max, float min);

static inline float fuzzyMFOutput(float x, float max, float min)
{
  return x * (max - min) + min;
}


/*
x = ((val - min) / (max - min));

y = x*(max - min) + min
Test: 
  max = 10;
  min = -10;

  x = 0.5;
  y = 0.5 * (10 - -10) + -10
  y = 0.5 * (20) - 10
  y = 10 - 10
  y = 0
  
  x = 1
  y = 1 * (10 - -10) + -10
  y = 20 - 10
  y = 10
*/

#endif 