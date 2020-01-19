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

#define PWM_MOTORS_INSTANCE  0 

#define MOTORS_PWM_TOP_VAL   2000
#define MOTORS_PWM_IDLE_VAL  1000

#define MOTORS_NUM   4

typedef struct {
    const int16_t topValueCount;
    
} motorsConfig_t;

/**@brief  */
typedef union 
{
  int16_t esc[4];
  int16_t esc1, esc2, esc3, esc4;
} motors_t;


error_t motorsSetup(void);

motors_t motorsMix(float throttle, float pitch, float roll, float yaw, float minOut, float maxOut);

error_t motorsDisable(void);

//                            DC1 BLeft    DC2 BRight    DC3 FRight      D4 FLeft
void motorsUpdate(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

void motorsUpdateAll(uint16_t dc);


#endif /* MPU60050H_H_ */