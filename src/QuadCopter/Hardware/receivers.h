/*
 * MPU60050h.h
 *
 * Created: 15/06/2019 14:38:49
 *  Author: Danlo
 */ 


#ifndef RECEIVERS_H_
#define RECEIVERS_H_

#include "error_codes.h"

/* Enable to print receiver values in endless loop */
#define RECEIVER_DEBUG_LOG 0

/* Setsup Hardware and starts reading receiver values */
error_t receiverSetup(void);

//TODO: Add functions for starting and stopping reading of 
//        receiver values.


/* Gets current Values from internal buffer */
uint32_t rc_get_roll(void);
uint32_t rc_get_pitch(void);
uint32_t rc_get_throttle(void);
uint32_t rc_get_yaw(void);
uint32_t rc_get_6_way_switch(void);
uint32_t rc_get_3_way_switch(void);


#endif /* MPU60050H_H_ */