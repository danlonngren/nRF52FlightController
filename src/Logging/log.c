#include "log.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "nrf_assert.h"


//-----------------------------------------------------------------------------
// Defines and variables
//-----------------------------------------------------------------------------

static sLogSetup * vLogSetup; // Log setup/init structure

// Logging level tags
const uint8_t strDebug[] =    "<DEBUG> ";
const uint8_t strInfo[] =     "<INFO> ";
const uint8_t strWarning[] =  "<WARNING> ";
const uint8_t strError[] =    "<ERROR> ";

// Lookup table for Sub System names
static uint8_t * const pSysNameTable[20] = {
  "SETUP",
  "CONTROL_LOOP",
  "FUZZY_LOGIC",
  "PID",
  "IMU",
  "MOTORS",
  "RECEIVER",
  "SENSORS",
  "TIMER",
  "SPI",
  "TWI",
  "BLE",
};


//-----------------------------------------------------------------------------
// NRF logging backend. May change for RTT Functions
//-----------------------------------------------------------------------------
#if 0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SENDLOG(...)    NRF_LOG_RAW_INFO( __VA_ARGS__)
#define FLUSHLOG()      NRF_LOG_FLUSH()
#endif


static void vLogPrint(uint8_t * pStr)
{
  if (vLogSetup->pFncPrintf != NULL)
    vLogSetup->pFncPrintf(pStr);
  else
    ASSERT(NULL);
}

//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------
static void strCombine(uint8_t target[], const uint8_t pStringIn[], uint8_t * pIndex, uint8_t size)
{
  while(*pStringIn != '\0' && target != NULL && pStringIn != NULL)
  {
    target[*pIndex] = *pStringIn;
    ++pStringIn;
    if (++(*pIndex) >= size) 
      break;
  }
}


 static void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }


 static void intToString(int32_t n, uint8_t s[], uint8_t *counter)
 {
     uint32_t i = 0, sign = n;
 
     if (sign < 0)  /* record sign */
         n = -n;          /* make n positive */
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
     *counter += i;
 }


//-----------------------------------------------------------------------------
// utility Functions
//-----------------------------------------------------------------------------
void vLogOff(void)
{
  vLogSetup->state = false;
}


void vLogOn(void)
{
  vLogSetup->state = true;
}


void vLogSetLevel(eLogLevel level, eLogSubSystem sys)
{
  //vLogSetup->currentLevel = level;
  
  vLogSetup->subSysLevel[sys] = level;

}

eLogLevel eLogGetSysLevel(eLogSubSystem sys)
{
  return vLogSetup->subSysLevel[sys];
}

void vLogFlush(void) 
{ 
  // Not in use as logs are not deferred
  //FLUSHLOG(); 
}


/**@brief Finds 
 *
 * @returns pointer to start of file directory & null if none found
 */
static uint8_t *strFindKetWord(uint8_t pTargetString[], const uint8_t pKeyword[], uint8_t size)
{
  uint8_t counter = 0;
  uint8_t *pRetrunStr = NULL;
 
  while ((*(pTargetString) != '\0') || counter >= size)
  {
    pTargetString++;
    counter++;

    const uint8_t *_pKeyword = pKeyword; // Create local pointer to keyword
     
    // Check if characters match
    if (*pTargetString == *_pKeyword) 
    { 
      pRetrunStr = pTargetString; // Set return pointer to first matching character
      bool keywordFound = false; // Variable for checking if match is found
      // Cycle through until strings no longer match or keyword ends '\0'
      while ((*pTargetString == *_pKeyword) || (counter >= size))
      {
        pTargetString++;
        _pKeyword++;
        counter++;
        // Check for keyword null terminator
        if (*(_pKeyword) == '\0')
        {
           keywordFound = true; // Set keyword found true
           break; // Break out of loop
        }
      }
      // Check if key word found else reset start string
      if (keywordFound == true)
      {
        break;
      }
      else  
      {
        pRetrunStr = NULL;
      }
    }
  }
  return pRetrunStr;
}



//-----------------------------------------------------------------------------
// Logging Functions
//-----------------------------------------------------------------------------
void vLog(eLogSubSystem sys, eLogLevel level, uint8_t * pMsg, uint8_t *pFilePath, uint32_t lineNumber)
{
  // Only continue if enabled and level is less than set current level
  if (vLogSetup->state == false || level > eLogGetSysLevel(sys) || pMsg == NULL)
    return;

  //vLogPrint(pFilePathDir);
  static uint8_t buffer[VLOG_MSG_MAX_CHAR];
  memset(buffer, 0, VLOG_MSG_MAX_CHAR);
  uint8_t charCount = 0;
  
  const uint8_t * levelString = NULL; // Temporary pointer pointing to log level string 
  switch (level) {
    case ELOG_LEVEL_DEBUG:    levelString = strDebug; break;
    case ELOG_LEVEL_INFO:     levelString = strInfo; break;
    case ELOG_LEVEL_WARNING:  levelString = strWarning; break;
    case ELOG_LEVEL_ERROR:    levelString = strError; break;
    default: break;
  }

  strCombine(buffer, levelString, &charCount, VLOG_MSG_MAX_CHAR);
  
  uint8_t lineNumArr[6];
  intToString(lineNumber, &(buffer[charCount]), &charCount);

  buffer[charCount++] = ':';
  uint8_t *pFilePathDir = strFindKetWord(pFilePath, vLogSetup->pSrcDirName, VLOG_MSG_MAX_CHAR);
  strCombine(buffer, pFilePathDir, &charCount, VLOG_MSG_MAX_CHAR);

  buffer[charCount++] = ' ';
  buffer[charCount++] = '-';
  buffer[charCount++] = ' ';

  uint8_t * pSys = pSysNameTable[sys];
  strCombine(buffer, pSys, &charCount, VLOG_MSG_MAX_CHAR);
  buffer[charCount++] = '>';
  buffer[charCount++] = ' ';

  strCombine(buffer, pMsg, &charCount, VLOG_MSG_MAX_CHAR);

  buffer[charCount] = '\0'; // Add Null terminator
  vLogPrint(buffer);  // Print string
} // vLog() End


//void vLog(eLogSubSystem sys, eLogLevel level, uint8_t * pMsg)
//{
//  // Only continue if enabled and level is less than set current level
//  if (vLogSetup->state == false || level > eLogGetSysLevel(sys) || pMsg == NULL)
//    return;
//
//  static uint8_t buffer[VLOG_MSG_MAX_CHAR];
//  memset(buffer, 0, VLOG_MSG_MAX_CHAR);
//  uint8_t charCount = 0;
//
//  vLogAddSysAndLevelStr(buffer, &charCount, sys, level);
//
//  strCombine(buffer, pMsg, &charCount, VLOG_MSG_MAX_CHAR);
//
//  buffer[charCount] = '\0'; // Add Null terminator
//  vLogPrint(buffer);  // Print string
//} // vLog() End
//


void vLogWithNum(eLogSubSystem sys, eLogLevel level, uint8_t * msg, uint32_t number, uint8_t *pFilePath, uint32_t lineNumber)
{
  if (vLogSetup->state == false || msg == NULL)
    return;

  uint8_t *pMsg = msg;
  uint8_t buffer[VLOG_MSG_MAX_CHAR], bufferIndex = 0; 
  memset(buffer, 0, VLOG_MSG_MAX_CHAR);
  
  while (*(pMsg) != '\0' && bufferIndex < VLOG_MSG_MAX_CHAR) 
  {
    if (*pMsg == '%')
    {
      if (*(++pMsg) == 'i')
      {       
        intToString(number, &(buffer[bufferIndex]), &bufferIndex);
        ++pMsg;
      } else break;
    }
    buffer[bufferIndex++] = *pMsg;
    ++pMsg;
  }

  vLog(sys, level, buffer, pFilePath, lineNumber);
}

void vLogWithFloat(eLogSubSystem sys, eLogLevel level, uint8_t * msg, float number, uint8_t *pFilePath, uint32_t lineNumber)
{
  if (vLogSetup->state == false || msg == NULL)
    return;

  uint8_t * pFMarker = "%1s%3d.%02d";
  float val = number;

  uint8_t buffer[VLOG_MSG_MAX_CHAR], bufferIndex = 0; 
  memset(buffer, 0, VLOG_MSG_MAX_CHAR);

  uint8_t *pMsg = msg; 
  while (*(pMsg) != '\0' && bufferIndex < VLOG_MSG_MAX_CHAR) 
  {
    if (*pMsg == '%')
    {
      if (*(++pMsg) == 'f')
      {       
        buffer[bufferIndex++] = (((val) < 0 && (val) > -1.0) ? ' ' : '-');
        
        int32_t fBody = (int32_t)(val < 0 ? val * -1 : val);
        intToString(fBody, &(buffer[bufferIndex]), &bufferIndex);
        
        buffer[bufferIndex++] = '.'; 

        int32_t fDec1 = (int32_t)((((val) > 0) ? (val) - (int32_t)(val) : (int32_t)(val) - (val))*100);
        intToString(fDec1, &(buffer[bufferIndex]), &bufferIndex);
        ++pMsg;
      } else break;
    }
    buffer[bufferIndex++] = *pMsg;
    ++pMsg;
  }

  vLog(sys, level, buffer, pFilePath, lineNumber);
}


//-----------------------------------------------------------------------------
// Initializer
//-----------------------------------------------------------------------------
uint32_t vLogInit(sLogSetup *logInit)
{
    vLogSetup = logInit; // Assign local pointer

    vLogOn(); // Enable Logs
   
    uint8_t arrTest[6];
    vLogSetLevel(ELOG_LEVEL_INFO, ELOG_SYS_PID);
    VLOG(ELOG_SYS_PID, ELOG_LEVEL_INFO, "Testing \r\n");
//    VLOG(ELOG_SYS_PID, ELOG_LEVEL_INFO, "ELOG_LEVEL_INFO \r\n");
//    VLOG(ELOG_SYS_PID, ELOG_LEVEL_DEBUG, "ELOG_LEVEL_DEBUG \r\n");
//    VLOG(ELOG_SYS_PID, ELOG_LEVEL_WARNING, "ELOG_LEVEL_WARNING \r\n");
//    VLOG_WITH_FLOAT(ELOG_SYS_PID, ELOG_LEVEL_ERROR, "ELOG_LEVEL_ERROR \r\n", 1.234f);
    
    uint8_t target[] = "thas.is.a.test.h";
    uint8_t keyWord[] = "a.";

    uint8_t * result = strFindKetWord(target, keyWord, sizeof(target));
    
    printf("Char: %s \r\n", result);

}


