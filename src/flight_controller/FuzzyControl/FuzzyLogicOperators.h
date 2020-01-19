#ifndef FUZZYLOGICOPERATORS_H
#define FUZZYLOGICOPERATORS_H

#include <stdint.h>


static inline float fuzzyMinf(float a, float b)
{
  if (a > b)
    return b;
  return a;
}

static inline float fuzzyMaxf(float a, float b)
{
  if (a < b)
    return b;
  return a;
}

static inline float fuzzyAND(float x, float y)
{
  return fuzzyMinf(x, y);
}


static inline float fuzzyOR(float x, float y)
{
  return fuzzyMaxf(x, y);
}


static inline float fuzzyPAND(float x, float y)
{
  return x * y;
}


static inline float fuzzyPOR(float x, float y)
{
  return ((x + y) - (x * y));
}

static inline float fuzzyNOT(float x)
{
  return (1.0f - x);
}



#endif 