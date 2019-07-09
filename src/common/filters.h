#ifndef FILTERS_H_
#define FILTERS_H_


#define FILTER_COMP(nd, ld, gain) ((nd * gain) + (ld *(1 - gain)))


#define FILTER_SINGLE_POLE_LP(nd, ld, gain) ((nd * gain) + (ld *(1 - gain)))

static inline float filterSinglePoleLowPass(float *lastVal, float *newVal, const float gain)
{
    float retVal = 0;
    *lastVal += gain * (*newVal - *lastVal);
    return *lastVal;
}

#endif