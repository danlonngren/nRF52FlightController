#include "testing.h"
#include "nrf_port.h"

#include "board_config.h"

void test_case(bool test, uint8_t stringIn[], uint8_t * pFile, int32_t lineNum)
{
  int32_t timeout = 0;
  uint8_t character = 0;
  uint8_t backslashIndex, dotIndex;
  uint8_t fileName[20];

  while ((++timeout) < 200 || character == '.')
  {
    character = pFile[timeout];

    if (character == '\\')
      backslashIndex = timeout;
    if (character == '.')
    {
      dotIndex = timeout;
      break;
    }
  }
  
  uint8_t fileNameLen = dotIndex - backslashIndex + 1;

  // Make sure file name plus 3 for file extension will not exceed array size (20 uint8_t)
  uint8_t len = ((fileNameLen) < 20) ? (fileNameLen) : 20;
  
  for (uint8_t i = 0; i < len; i++)
  {
      fileName[i] = pFile[backslashIndex + i + 1];
  }
  
  LOG("File: %s - %i \r\n", fileName, lineNum);
  FLUSH();
}
