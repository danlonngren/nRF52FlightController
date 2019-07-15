#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include "error_codes.h"

void timer0Setup(void);


int16_t getTimeUs(void);


#endif