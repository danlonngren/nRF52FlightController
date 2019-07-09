/*
 * quad_dynamics.c
 *
 * Magwick filter and fusion scheme
 *
 * https://github.com/kriswiner/GY-80/blob/master/GY80BasicAHRS.ino
 *
 * Created: 15/06/2019 17:30:59
 *  Author: Danlo
 */ 



#include "quaternion_math.h"

#include "common_math.h"

#include "vector_math.h"

#include "nrf_port.h"


#ifndef NULL
#define NULL  0
#endif

