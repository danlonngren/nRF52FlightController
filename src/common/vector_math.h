#ifndef VECTOR_MATH
#define VECTOR_MATH

//#define X 0
//#define Y 1
//#define Z 2
#include "common_math.h"

typedef union {
    float v[3];
    struct {
       float x,y,z;
    };
} vector3_t;


typedef struct {
    float m[3][3];
} matrix3_t;


typedef struct {
    vector3_t axis;
    float angle;
} axisAngle_t;


static inline vector3_t * rotationMatrixRotateVector3(vector3_t * result, const vector3_t * a, const matrix3_t * rmat)
{
    vector3_t r;

    r.x = rmat->m[0][0] * a->x + rmat->m[1][0] * a->y + rmat->m[2][0] * a->z;
    r.y = rmat->m[0][1] * a->x + rmat->m[1][1] * a->y + rmat->m[2][1] * a->z;
    r.z = rmat->m[0][2] * a->x + rmat->m[1][2] * a->y + rmat->m[2][2] * a->z;

    *result = r;
    return result;
}


static inline float vector3NormSquare(const vector3_t * v)
{
    return (v->x*v->x) + (v->y*v->y) + (v->z*v->z);
}

static inline vector3_t * vector3Normalize(vector3_t * result, const vector3_t * v)
{
    float length = sqrtf(vector3NormSquare(v));
    if (length != 0) {
        result->x = v->x / length;
        result->y = v->y / length;
        result->z = v->z / length;
    }
    else {
        result->x = 0;
        result->y = 0;
        result->z = 0;
    }
    return result;
}

static inline vector3_t * vector3CrossProduct(vector3_t * result, const vector3_t * a, const vector3_t * b)
{
    vector3_t vTemp;

    vTemp.x = a->y * b->z - a->z * b->y;
    vTemp.y = a->z * b->x - a->x * b->z;
    vTemp.z = a->x * b->y - a->y * b->x;

    *result = vTemp;
    return result;
}


static inline vector3_t * vector3Add(vector3_t * result, const vector3_t * a, const vector3_t * b)
{
    vector3_t vTemp;

    vTemp.x = a->x + b->x;
    vTemp.y = a->y + b->y;
    vTemp.z = a->z + b->z;

    *result = vTemp;
    return result;
}

static inline vector3_t * vector3Subtract(vector3_t * result, const vector3_t * a, const vector3_t * b)
{
    vector3_t vTemp;

    vTemp.x = a->x - b->x;
    vTemp.y = a->y - b->y;
    vTemp.z = a->z - b->z;

    *result = vTemp;
    return result;
}

static inline vector3_t * vector3Multiply(vector3_t * result, const vector3_t * a, const vector3_t * b)
{
    vector3_t vTemp;

    vTemp.x = a->x * b->x;
    vTemp.y = a->y * b->y;
    vTemp.z = a->z * b->z;

    *result = vTemp;
    return result;
}

static inline vector3_t * vector3Scale(vector3_t * result, const vector3_t * a, const float b)
{
    vector3_t vTemp;

    vTemp.x = a->x * b;
    vTemp.y = a->y * b;
    vTemp.z = a->z * b;

    *result = vTemp;
    return result;
}


static inline vector3_t * vector3Init(vector3_t * target, const float x, const float y, const float z)
{
    vector3_t vTemp;

    vTemp.x = x;
    vTemp.y = y;
    vTemp.z = z;

    *target = vTemp;
    return target;
}
#endif