#ifndef TWI_H_
#define TWI_H_


#include <stdint.h>
#include "error_codes.h"


error_t twiInit(uint8_t scl, uint8_t sda, uint8_t irqPrio);

error_t twiReadData(uint8_t address, uint8_t reg, uint8_t *rxOut, uint8_t size);

error_t twiWriteData(uint8_t address, uint8_t reg, uint8_t *txIn, uint8_t size);


#endif