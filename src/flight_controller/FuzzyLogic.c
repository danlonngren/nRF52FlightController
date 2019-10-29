#include "FuzzyLogic.h"

#include <stdint.h>
#include <stdbool.h>
#include "common_math.h"

#include "nrf_port.h"

typedef struct {
  float       outputGain;
  float       rateGain;
  uint32_t    outputMax;
  
  float       eLimit;
  float       deLimit;
  float       reLimit;
  float       gLimit;
  
  float       sOutGain;

} fuzzyData_t;



long funcPyth(long x, long y, long z)
{
return sqrt((x * x) + (y * y) + (z * z));
}

int funcConvertStick(int in) {
  int in2 = in - 1500;
  int out = 0;

  if (in > 1500)
     out = ((in2 * in2) / 1000 + (in2) / 2 + 1500);
  else if (in < 1500)
     out =  (1500 - (in2 * in2) / 1000 + in2 / 2);
 return out;
}

int funcConvertThrottle(int in) {
  int Throttle = sqrt(in - 1000) * 31.63 + 1000;
  return Throttle;
}

float funcGauss(float  x, float a)
{
  float y;
  y = exp((-1.0f * (x * x)) / (2.0f * a)); //centre is 0
  return y;
}


float funcLinear(float x, float max)
{
  return (max + x) / (2.0f * max); // centre is 0.5
}


float funcpositiveNL(float x)//from -10 to 10
{
  float y;
  y = (((x * x * x + x) / 2020.0) + 0.5); // centre is 0.5
  return y;
}
float funcnegativeNL(float x)//from -10 to 10
{
  float y;
  y = ((((-x * x * x) - x) / 2020.0) + 0.5); // centre is 0.5
  return y;
}


float deFuzzyifierCentroid(float * u, float * a, int32_t size)
{
  float retVal = 0;
  float dem = 0;
  float num = 0;
   
  while ((--size) >= 0)
  {
    dem += *a;
    a++;

    num += *u;
    u++;
  }

//  float fuzzy_output = ((u[1] + u[2] + u[3] + u[4] + u[5] + u[6] + u[7] + u[8]) / (a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8])); //Centroidal Defuzzifier  
  return (num / dem);
}


typedef struct {
  float error;
  float errorLast;
  float errorDiff;
  float errorSum;
} fuzzyInputs_t;

float FuzzyController(fuzzyInputs_t * inputs, fuzzyData_t * constants, bool level) {
   
  float w[5];

  if (level) {
    w[1] = 1.0;
    w[2] = 0.35;
    w[3] = 1.0;
    w[4] = 0.7;
  } if (!level) {
    w[1] = 1.3;
    w[2] = 0.15;
    w[3] = 1.0;
    w[4] = 0;
  }

  float e_last = 1;
  float rate_gain = 1;
  // Process Inputs to useful signals ie. error, error difference, and error rate (PID)
  inputs->errorSum += inputs->error * rate_gain;  
  inputs->errorDiff = (inputs->error - e_last);
  inputs->error     = mathConstrain(inputs->error, -1 * constants->eLimit, constants->eLimit); //Limits the output of inputs->error
  inputs->errorSum  = mathConstrain(inputs->errorSum, -1 * constants->reLimit, constants->reLimit);
  inputs->errorDiff = mathConstrain(inputs->errorDiff, -1 * constants->deLimit, constants->deLimit);

  // Translate into fuzzy values ranging from 0 to 1;
  float pos_e = funcLinear(inputs->error, constants->eLimit); //linear posetive function (centre point 0.5 with output from 0 to 1)
  float pos_d = funcLinear(inputs->errorDiff, constants->deLimit);
  float pos_r = funcLinear(inputs->errorSum, constants->reLimit);

  // Neg side
  float neg_e = funcLinear((-1) * inputs->error, constants->eLimit);
  float neg_d = funcLinear((-1) * inputs->errorDiff, constants->deLimit);
  float neg_r = funcLinear((-1) * inputs->errorSum, constants->reLimit);

  float gau_e = funcGauss(inputs->error, constants->gLimit); //Gaussian function with center point at 0
  // gauD = funcgauss(inputs->errorDiff, g);

  float a[9];
  a[1] = neg_e * neg_d; //r1  //rule 1 and 2 acts as the Proportional part from a PID
  a[2] = pos_e * pos_d; //r2  //
  a[3] = neg_e * pos_d; //r3  //rule 3 and 4 acts as the derivative part from a PID
  a[4] = pos_e * neg_d; //r4  //
  a[5] = neg_e * neg_r; //r5  //rule 5 and 6 acts as the integral part from a PID
  a[6] = pos_e * pos_r; //r6  //

 // a[7] = gau_e * pos_d; //r7  //gaussian function rule further reducing overshoot - places more weight on D term when gauE = 1
 // a[8] = gau_e * neg_d; //r8  //

  //output MF
  float u[9];
  u[1] = (a[1] * w[1] * -1)  * constants->sOutGain;          //h - Singleton output (can be replaced with function)
  u[2] = (a[2] * w[1] *  1)  * constants->sOutGain;
  u[3] = (a[3] * w[2] *  1)  * constants->sOutGain;
  u[4] = (a[4] * w[2] * -1)  * constants->sOutGain;
  u[5] = (a[5] * w[3] * -1)  * constants->sOutGain;
  u[6] = (a[6] * w[3] *  1)  * constants->sOutGain;
  //u[7] = (a[7] * w[4] *  1)  * constants->sOutGain;
  //u[8] = (a[8] * w[4] * -1)  * constants->sOutGain;

  float fuzzy_output = ((u[1] + u[2] + u[3] + u[4] + u[5] + u[6] + u[7] + u[8]) / (a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8])); //Centroidal Defuzzifier
  
  float output_gain = 1;

  fuzzy_output *= output_gain;

  float out_max = 1;
  fuzzy_output = mathConstrain(fuzzy_output, out_max, (-1)*out_max);
 
  inputs->errorLast = inputs->error;

  return fuzzy_output;
}



uint32_t fuzzyTest(void)
{
  uint32_t pass = 0;

  // Function Testing

  LOG("deFuzzyifierCentroid Test \r\n");

  float u[3] = {0.5,0.5,0.5};
  float a[3] = {0.5,0.5,0.5};
  float result = deFuzzyifierCentroid(u, a, sizeof(u)/sizeof(float));
  utilPrintFloat("Test1: ", result);

  if (result == 1)
    pass = true;

  LOG("funcLinear Test \r\n");
  result = funcLinear(0, 100);
  utilPrintFloat("Test1: ", result);

  if (result == 0.5f)
    pass = true;

  result = funcLinear(50, 100);
  utilPrintFloat("Test2: ", result);

   if (result == 0.75f)
    pass = true;

  result = funcLinear(-50, 100);
  utilPrintFloat("Test3: ", result);

   if (result == 0.25f)
    pass = true;

  return pass;

}