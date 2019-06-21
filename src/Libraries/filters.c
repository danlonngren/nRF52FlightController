#include "filters.h"

#include <math.h>



float filterComplementrary(float lastData, float newData, float gain)
{
    float gain1 = 1 - gain;
    return (lastData * gain) + (newData * gain1);
}