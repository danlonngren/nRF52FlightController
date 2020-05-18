#include "log.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "nrf_assert.h"

#include "strings.h"

VLOG_MODULE_INIT("LOG", VLOG_DEBUG_LEVEL);


void utilPrintFloatArray(uint8_t *string, float *arr, uint8_t size)
{
  LOG("%s ", string);
  for (uint8_t i = 0; i < size; i++)
  {
  
    LOG("%i: "FM" ", i, FW(arr[i]));
  }
  LOG("\r\n");
}

void utilPrintXYZ(uint8_t *name, int32_t x, int32_t y, int32_t z)
{
    LOG("%s X:%5i, Y:%5i, Z:%5i\n", name, x, y, z);
}

void utilPrintFloat(uint8_t * string, float val)
{ 
    LOG("%s" FM" \r\n", string, FW(val));
}




//-----------------------------------------------------------------------------
// Defines and variables
//-----------------------------------------------------------------------------

const uint8_t *levelStrArr[5] = {
  "NONE",
  "WARNING",
  "ERROR",
  "INFO",
  "DEBUG"
};
#include "nrf_delay.h"
static void log_print_format(eLogLevel level, int32_t lineNum, const char *name)
{
  printf("[%s][%3i:%s] ", levelStrArr[level], lineNum, name);
  nrf_delay_us(900);
}

void log_with_float(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, float num)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  printf(msg, FW(num));
  printf("\r\n");
}

void log_with_int(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, int num)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  printf(msg, num);
  printf("\r\n");
}

void log0(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  printf(msg);
  printf("\r\n");
}

void logN(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, ...)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  va_list args;
  va_start(args, msg);
  vprintf(msg, args);
  va_end(args);
  printf("\r\n");
}


/**@brief Structure for initializing and storing logging data. */
typedef struct {
  _Bool  state;                         //!< Logging state
}sLogSetup;

static sLogSetup vLogSetup; // Log setup/init structure

void vLogOff(void)
{
  vLogSetup.state = false;
}


void vLogOn(void)
{
  vLogSetup.state = true;
}


bool vLogState(void)
{
  return vLogSetup.state;
}

//-----------------------------------------------------------------------------
// Initializer
//-----------------------------------------------------------------------------
uint32_t vLogInit(void)
{
    vLogOn(); // Enable Logs
   
//    uint8_t arrTest[6];
//
//    vLOG(vLOG_LEVEL_WARNING, "Test Float: "FM"", FW(1.123));
//    
//    vLOG(vLOG_LEVEL_WARNING, "Test1");
//    vLOG(vLOG_LEVEL_WARNING, "Test2: %i", 123);
//    vLOG(vLOG_LEVEL_WARNING, "Test2: %i", 123);
//
//    vLOG_FLOAT(vLOG_LEVEL_INFO, "Float value test "FM"", -1.124);

    return true;
}

