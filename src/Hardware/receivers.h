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

#define GPIOTE_REVEIVER_CH  0
#define TIMER_RECEIVER_CH   0
#define PPI_RECEIVER_CH     0

#define RECEIVER_PPM_INPUT_MAX  2100

#define RECEIVER_CENTER         1500

#define RECEIVER_3WAY_POS1_MAX  RECEIVER_PPM_INPUT_MAX
#define RECEIVER_3WAY_POS1_LOW  1800

#define RECEIVER_3WAY_POS2_MAX  RECEIVER_3WAY_POS1_LOW
#define RECEIVER_3WAY_POS2_LOW  1400

#define RECEIVER_3WAY_POS3_MAX  RECEIVER_3WAY_POS2_LOW
#define RECEIVER_3WAY_POS3_LOW  900
typedef enum
{
  RC_SWITCH_3WAY_OFF,
  RC_SWITCH_3WAY_OFF_TO_ON,
  RC_SWITCH_3WAY_ON,
  RC_SWITCH_3WAY_ON_TO_OFF,
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

void receiverEnable(void);

void receiverDisable(void);

receivers_t rcGetChannels(void);

rcSwitch3Way_t rcSwitchGet3Way(void);

//rcSwitch6Way_t rcSwitchGet6Way(void);

uint16_t get_16bit_diff_tick(uint16_t test_tick, uint16_t prev_tick);

#endif /* MPU60050H_H_ */