#include "nrf_port.h"

void utilPrintFloatArray(uint8_t *string, float *arr, uint8_t size)
{
  LOG("%s ", string);
  for (uint8_t i = 0; i < size; i++)
  {
  
    LOG("%i:" NRF_LOG_FLOAT_MARKER" ", i, NRF_LOG_FLOAT(arr[i]));
    FLUSH();
  }
  LOG("\r\n");
}

void utilPrintXYZ(uint8_t *name, int32_t x, int32_t y, int32_t z)
{
    LOG("%s X:%5i, Y:%5i, Z:%5i\n", name, x, y, z);
}

void utilPrintFloat(uint8_t * string, float val)
{ 
    LOG("%s" NRF_LOG_FLOAT_MARKER" \r\n", string, NRF_LOG_FLOAT(val));
}