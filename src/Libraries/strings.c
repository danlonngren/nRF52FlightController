#include "strings.h"

#include "stdint.h"
#include "stdbool.h"


#ifndef NULL
#define NULL 0
#endif


/**@brief Finds 
 *
 * @returns pointer to start of file directory & null if none found
 */
uint8_t *strFindKeyWord(const uint8_t pTarget[], int32_t targetSize, const uint8_t pKey[], int32_t keySize)
{
  if (pTarget == NULL || pKey == NULL)
   return NULL;

  uint8_t *pReturn = NULL; // Pointer to start of keyword
  int32_t targetCounter = 0, kwCounter = 0;
  bool kwFound = false;
  
  for (targetCounter = 0; (targetCounter < targetSize); targetCounter++)
  {
    if (pTarget[targetCounter] == pKey[kwCounter])
    {
      if (pReturn == NULL)
        pReturn = &pTarget[targetCounter]; // Set return pointer to first matching character

      // Check if end of keyword
      if (++kwCounter >= keySize)
      {
        // Set keyword found and break
        kwFound = true; 
        break;
      }
    }
    else
    {
      // Reset
      pReturn = NULL;
      kwCounter = 0;
    }
  }

  // Prevent return half keyword if targetCounter >= targetSize
  if (kwFound == false)
    pReturn = NULL;

  return pReturn;
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


static void strCopy(uint8_t target[], const uint8_t pStrIn[], uint8_t len)
{
  if (target == NULL || pStrIn == NULL)
    return;

  for (uint8_t idx = 0; (idx < len); idx++)
  { 
    target[idx] = pStrIn[idx];
  }
}


 static void strReverse(uint8_t s[], int32_t len)
 {
     int32_t i, j;
     for (i = 0, j = len-1; i<j; i++, j--) {
         uint8_t c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }

 static int32_t strLen(const uint8_t s[])
 {
    int32_t idx = 0;
    while(idx < 400)
    {
      if (s[idx++] == '\0')
        return idx;
    
    }
    return -1;
 }


 static void intToString(uint8_t s[], int32_t value, uint8_t *counter)
 {
     uint32_t i = 0, sign = value;
 
     if (sign < 0)  /* record sign */
     {
         value = -value;          /* make n positive */
     }
     do {       /* generate digits in reverse order */
         s[i++] = value % 10 + '0';   /* get next digit */
     } while ((value /= 10) > 0);     /* delete it */
     if (sign < 0)
     {
         s[i++] = '-';
     }
     s[i] = '\0';
     strReverse(s, i);
     *counter += i;
 }


/**@brief Finds word in string
 *
 * @returns pointer to start of found word or NULL if none found
 */
static const int32_t logStrFindWord(const uint8_t pTarget[], int32_t targetSize, const uint8_t pKey[], int32_t keySize)
{
  if (pTarget == NULL || pKey == NULL)
   return -1;

  int32_t pReturn = 0; // Pointer to start of keyword
  int32_t kwCounter = 0;

  for (int32_t targetCounter = 0; (targetCounter < targetSize); targetCounter++)
  {
    if (pTarget[targetCounter] == pKey[kwCounter])
    {        
      // Check if end of keyword
      if (++kwCounter >= keySize)
      {
        pReturn = targetCounter - kwCounter + 1; // Set return pointer to first matching character
        break;
      }
    }
    else kwCounter = 0;
  }

  return pReturn;
}
