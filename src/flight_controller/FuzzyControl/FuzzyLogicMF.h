#ifndef FUZZYLOGICMF_H
#define FUZZYLOGICMF_H

#include <stdint.h>

float funcGauss(float  x, float a);

float fuzzyMFLinearPos(float val, float max, float min);

float fuzzyMFLinearNeg(float val, float max, float min);

float fuzzyMFGauss(float  x, float max, float min);

float fuzzyMFGaussInv(float x, float max, float min);

#endif 