#include "log.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "nrf_assert.h"

#include "strings.h"

VLOG_MODULE_INIT("LOG", VLOG_DEBUG_LEVEL);

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

static void log_print_format(eLogLevel level, int32_t lineNum, const char *name)
{
  printf("[%s][%3i:%s] ", levelStrArr[level], lineNum, name);
}

void log_with_float(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, float num)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  printf(msg, FW(num));
  printf("\n");
}

void log_with_int(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, int num)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  printf(msg, num);
  printf("\n");
}

void log0(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg)
{
  if (level > type->level || vLogState() != true)
    return;

  log_print_format(level, lineNum, type->pName);
  printf(msg);
  printf("\n");
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
  printf("\n");
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
   
    uint8_t arrTest[6];

    vLOG(ELOG_LEVEL_WARNING, "Test Float: "FM"", FW(1.123));
    
    vLOG(ELOG_LEVEL_WARNING, "Test1");
    vLOG(ELOG_LEVEL_WARNING, "Test2: %i", 123);
    vLOG(ELOG_LEVEL_WARNING, "Test2: %i", 123);

    vLOG_FLOAT(ELOG_LEVEL_INFO, "Float value test "FM"", -1.124);

    return true;
}

