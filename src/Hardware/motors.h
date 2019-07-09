/*
 * MPU60050h.h
 *
 * Created: 15/06/2019 14:38:49
 *  Author: Danlo
 */ 


#ifndef MOTORS_H_
#define MOTORS_H_

#include "error_codes.h"
#include <stdbool.h>


typedef struct 
{
  int16_t esc1;
  int16_t esc2;
  int16_t esc3;
  int16_t esc4;
} esc_t;

typedef struct 
{
  int16_t esc[4];
} motorOutput_t;


error_t motorsSetup(void);

esc_t motorsMix(float throttle, float pitch, float roll, float yaw, float minOut, float maxOut);

error_t motorsDisable(void);



//                            DC1 BLeft    DC2 BRight    DC3 FRight      D4 FLeft
void pwm_update_duty_cycle(uint16_t dc1, uint16_t dc2, uint16_t dc3, uint16_t dc4);

void pwm_update_duty_cycle_all(uint16_t dc);



#endif /* MPU60050H_H_ */