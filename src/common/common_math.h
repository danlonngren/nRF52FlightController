#ifndef COMMON_MATH_H
#define COMMON_MATH_H

#include <stdint.h>
#include <stdbool.h>

#include "nordic_common.h"
#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#define g 9.81f // 1g ~ 9.81 m/s^2

#ifndef PI
#define PI  3.14159265358979323846f
#endif

#define TO_RADIANS   PI/180.0f
#define TO_DEGREES   180.0f/PI

#include <math.h>

float acos_approx(float x);

float atan2_approx(float y, float x);


static inline float toRadians(float degrees)
{
    return degrees * (PI / 180.0f);
}   

static inline float toDegrees(float radians)
{
    return radians * (180.0f / PI);
}    


static inline float invSqrt(float x) 
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static inline float mathConstrain(float target, float min, float max)
{
    if (target > max) target = max;
    else if (target < min) target = min;
    return target;
}

#define US_TO_SECONDS(x) (x / 1000000.0f)


/*-----------------Bit Operations----------------*/
static inline int16_t uint8toInt16(uint8_t H, uint8_t L)
{
    return ((int16_t)H << 8) | L;
}



#endif

