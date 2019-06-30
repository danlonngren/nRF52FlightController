#ifndef COMMON_MATH_H
#define COMMON_MATH_H

#include <stdint.h>

#define g 9.81f // 1g ~ 9.81 m/s^2

#ifndef PI
#define PI  3.14159
#endif

#include <math.h>

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


#define US_TO_SECONDS(x) (x / 1000000.0f)


/*-----------------Bit Operations----------------*/
static inline int16_t uint8toInt16(uint8_t H, uint8_t L)
{
    return ((int16_t)H << 8) | L;
}



#endif

