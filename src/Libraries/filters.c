#include "filters.h"

#include <math.h>



/**@breif Simple complementary filter.
 *
 * Gain is 1 - gain and applied to new data sample. A
 *  higher value will result in less filtering.
 *
 * param[in] currentData  Current filtered data
 * param[in] newData      New data in.
 * param[in] gain         Gain from 1 to 0
 */
float filterComplementary(float newData, float lastData, float newGain)
{
    return (newData * newGain) + (lastData * (1 - newGain));
}

