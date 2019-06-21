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

#define CH_GPIOTE  0
#define CH_TIMER   0


typedef enum
{
  RC_SWITCH_3WAY_OFF,
  RC_SWITCH_3WAY_ON,
  RC_SWITCH_3WAY_ONPLUS,
} rcSwitch3Way_t;

typedef struct
{
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
} receivers_t;


/* Setsup Hardware and starts reading receiver values */
error_t receiverSetup(void);

receivers_t rcGetChannels(void);

rcSwitch3Way_t rcSwitchGet3Way(void);
//rcSwitch6Way_t rcSwitchGet6Way(void);


//TODO: Add functions for starting and stopping reading of 
//        receiver values.

uint16_t get_16bit_diff_tick(uint16_t test_tick, uint16_t prev_tick);

/* Gets current Values from internal buffer */
uint32_t rc_get_roll(void);
uint32_t rc_get_pitch(void);
uint32_t rc_get_throttle(void);
uint32_t rc_get_yaw(void);
uint32_t rc_get_6_way_switch(void);
uint32_t rc_get_3_way_switch(void);


#endif /* MPU60050H_H_ */