#include "FuzzyLogicOperators.h"

#include <stdint.h>
#include <stdbool.h>
#include "common_math.h"


//-----------------------------------------------------------------------------
// Fuzzy Logical Operators
//-----------------------------------------------------------------------------

float fuzzyAND(float x, float y)
{
  return x * y;
}

float fuzzyOR(float x, float y)
{
  return ((x + y) - (x * y));
}


float fuzzyNOT(float x)
{
  return (1.0f - x);
}
