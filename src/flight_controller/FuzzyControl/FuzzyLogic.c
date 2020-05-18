#include "FuzzyLogic.h"

#include <stdint.h>
#include <stdbool.h>
#include "common_math.h"

#include "nrf_port.h"

#include "FuzzyLogicMF.h"
#include "FuzzyLogicOperators.h"

#include "testing.h"

#include "logging/inc/log.h"
VLOG_MODULE_INIT("fuzzyLogic", vLOG_LEVEL_DEBUG);



int32_t fuzzyAddSet(sFuzzySet *pSet, eMFType mf, float highLim, float lowLim)
{
  if (pSet == NULL)
    return -1;

  pSet->highLim   = highLim;
  pSet->lowLim    = lowLim;
  pSet->mfType    = mf;
  return 0;
}



float fuzzySugenoCalcOut(float *in1, float *in2, sFuzzySugenoConst c)
{
  if (in1 == NULL || in2 == NULL)
    return 0;

  return *in1 * c.a + *in2 * c.b + c.c;
}



///////////////////////////////////////////////////////////////////////////////
void fuzzyEvaluateMF(float input, rule_s * rule, float * outputWeighted, float *output)
{
  float temp = 0;
  
  ASSERT(outputWeighted != NULL);

  switch (rule->mfType)
  {
    case MF_LINEAR_POS:
      temp = fuzzyMFLinearPos(input, rule->highLim, rule->lowLim);
      break;
    case MF_LINEAR_NEG:
      temp = fuzzyMFLinearNeg(input, rule->highLim, rule->lowLim);
      break;     
    case MF_GAUSS:
      temp = fuzzyMFGaussInv(input, rule->highLim, rule->lowLim);
      break;
    default: break;
  }

  // Return unweighted member value if requested
  if (output != NULL)
    *output = temp;

  *outputWeighted = temp;
}



float FuzzyController(fuzzyInputs_s * inputs, fuzzyRules_s * rules) 
{
  // Process Inputs to useful signals ie. error, error difference, and error rate (PID)
 
  inputs->errorSum += inputs->error; 

  // Prevent integral windup
  inputs->errorSum = constrainF(inputs->errorSum,  rules->i_ruleP.highLim, rules->i_ruleP.lowLim); 
  inputs->errorDiff = (inputs->error - inputs->errorLast);
  inputs->errorLast = inputs->error;

  //-----------------------------------------------------------------------------
  // Fuzzify Inputs
  //-----------------------------------------------------------------------------
  // Translate into fuzzy values ranging from 0 to 1;
  float weightedMembersP[3], weightedMembersN[3];
  float membersP[3], membersN[3];
  fuzzyEvaluateMF(inputs->error,     &rules->p_ruleP, &weightedMembersP[0], &membersP[0]); //linear positive function (center point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &rules->d_ruleP, &weightedMembersP[1], &membersP[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &rules->i_ruleP, &weightedMembersP[2], &membersP[2]);

  // Neg side
  fuzzyEvaluateMF(inputs->error,     &rules->p_ruleN, &weightedMembersN[0], &membersN[0]); //linear positive function (center point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &rules->d_ruleN, &weightedMembersN[1], &membersN[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &rules->i_ruleN, &weightedMembersN[2], &membersN[2]);
 
  //-----------------------------------------------------------------------------
  // Apply Rules
  //-----------------------------------------------------------------------------
//  float fuzzyRules[6];
//  fuzzyRules[0] = weightedMembersN[0]; //r1 - rule 1 and 2 acts as the Proportional part from fuzzyRules PID
//  fuzzyRules[1] = weightedMembersP[1]); //r3 - rule 3 and 4 acts as the derivative part from fuzzyRules PID
//  fuzzyRules[2] = weightedMembersN[2]); //r5 - rule 5 and 6 acts as the integral part from fuzzyRules PID
//
//  fuzzyRules[3] = weightedMembersP[0]; //r2  
//  fuzzyRules[4] = fuzzyOR(weightedMembersP[0], weightedMembersN[1]); //r4  
//  fuzzyRules[5] = fuzzyOR(weightedMembersP[0], weightedMembersP[2]); //r6  

  //-----------------------------------------------------------------------------
  // Defuzzify
  //-----------------------------------------------------------------------------
  float wMemberP = 0;//weightedMembersP[0];
  float wMemberN = 0;//weightedMembersN[0];
  float memberSum = 0;
  for (uint32_t i = 0; i < 3; i++)
  {
    // Aggrigate using
    wMemberN += (weightedMembersN[i] * -10.0f);
    wMemberP += (weightedMembersP[i] * 10.0f);
    memberSum += (weightedMembersP[i] + weightedMembersN[i]);
  }
  return  (wMemberP + wMemberN) / memberSum;
}


float FuzzyControllerPIDTest(fuzzyInputs_s * inputs, fuzzyRules_s * rules) 
{

  // Prevent integral windup
  inputs->errorSum += inputs->error; 
  inputs->errorSum = constrainF(inputs->errorSum,  rules->i_ruleP.highLim, rules->i_ruleP.lowLim); 
  inputs->errorDiff = (inputs->error - inputs->errorLast);
  inputs->errorLast = inputs->error;
  

  // Process Inputs to useful signals ie. error, error difference, and error rate (PID)
  float sum = inputs->error * 1.4 + inputs->errorDiff *0.4  + inputs->errorSum*0.1;
  float out = 0;
  if (sum < 0)
    out = -1.0f;
  else  
    out = 1.0f;


  vLOG(vLOG_LEVEL_DEBUG, "Inputs: err: "FM", errSum: "FM", errDiff: "FM", errLast: "FM"", 
                                                  FW(inputs->error),
                                                  FW(inputs->errorSum),
                                                  FW(inputs->errorDiff),
                                                  FW(inputs->errorLast));
  //-----------------------------------------------------------------------------
  // Fuzzify Inputs
  //-----------------------------------------------------------------------------
  // Translate into fuzzy values ranging from 0 to 1;
  float weightedMembers[6];
  float members[6];
  fuzzyEvaluateMF(inputs->error,     &rules->p_ruleP, &weightedMembers[0], &members[0]); //linear posetive function (centre point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &rules->d_ruleP, &weightedMembers[1], &members[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &rules->i_ruleP, &weightedMembers[2], &members[2]);
  float pMembersSum = weightedMembers[0] + weightedMembers[1] + weightedMembers[2];
  // Neg side
  fuzzyEvaluateMF(inputs->error,     &rules->p_ruleN, &weightedMembers[3], &members[3]); //linear posetive function (centre point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &rules->d_ruleN, &weightedMembers[4], &members[4]);
  fuzzyEvaluateMF(inputs->errorSum,  &rules->i_ruleN, &weightedMembers[5], &members[5]);
  float nMembersSum = weightedMembers[3] + weightedMembers[4] + weightedMembers[5];
  

  //-----------------------------------------------------------------------------
  // Apply Rules
  //-----------------------------------------------------------------------------
  float fuzzyRules[6];
  fuzzyRules[0] = (weightedMembers[0]); //r1 - rule 1 and 2 acts as the Proportional part from fuzzyRules PID
  fuzzyRules[1] = fuzzyPAND(weightedMembers[0], weightedMembers[1]); //r1 - rule 1 and 2 acts as the Proportional part from fuzzyRules PID
  fuzzyRules[2] = fuzzyPAND(weightedMembers[0], weightedMembers[2]); //r3 - rule 3 and 4 acts as the derivative part from fuzzyRules PID

  fuzzyRules[3] = fuzzyPAND(weightedMembers[3], weightedMembers[1]); //r5 - rule 5 and 6 acts as the integral part from fuzzyRules PID
  fuzzyRules[4] = fuzzyPAND(weightedMembers[0], weightedMembers[2]); //r5 - rule 5 and 6 acts as the integral part from fuzzyRules PID
  fuzzyRules[5] = fuzzyPAND(weightedMembers[0], weightedMembers[5]); //r5 - rule 5 and 6 acts as the integral part from fuzzyRules PID

 
  //-----------------------------------------------------------------------------
  // Defuzzify
  //-----------------------------------------------------------------------------
  float wMember = 0;
  float memberSum = 0;
  float test = 0;
  for (uint32_t i = 0; i < 3; i++)
  {
    // Aggrigate using
    wMember += (fuzzyRules[i] * sum);
    vLOG(vLOG_LEVEL_DEBUG, "Member Result: "FM"", FW(fuzzyRules[i] * sum));
    memberSum += (weightedMembers[i]);
    test += fuzzyMFOutput(fuzzyRules[i], 20.0f, -20.0f);
  }
  
  float test1 = fuzzyRules[0] * inputs->error * 1.4 + 
                  fuzzyRules[1] * (inputs->error * 1.4 + inputs->errorDiff *0.4) +
                  fuzzyRules[2] * (inputs->error * 1.4 + inputs->errorSum*0.1);
        

  float retVal = (wMember) / memberSum;
  float retVal1 = (test1) / (fuzzyRules[0] + fuzzyRules[1] +fuzzyRules[2]);
  vLOG(vLOG_LEVEL_DEBUG, "ret: "FM"", FW(retVal));
  vLOG(vLOG_LEVEL_DEBUG, "ret1: "FM"", FW(retVal1));
  return retVal;
}

#define P_LIM 100.0f //250.0f
#define I_LIM 100.0f //30000.0f
#define D_LIM 100.0f //20.0f
#define O_LIM 100.0f //400.0f //20000.0f

#define P_WIGHT 1.0f
#define I_WIGHT 1.0f //2.6f
#define D_WIGHT 1.0f

// Global structure containing all fuzzy rules
static fuzzyRules_s fuzzyRulesS = {
 .p_ruleP = {P_LIM, -P_LIM, MF_GAUSS, P_WIGHT}, 
 .i_ruleP = {I_LIM, -I_LIM, MF_GAUSS, I_WIGHT},
 .d_ruleP = {D_LIM, -D_LIM, MF_GAUSS, D_WIGHT},

 .p_ruleN = {P_LIM, -P_LIM, MF_GAUSS, P_WIGHT}, 
 .i_ruleN = {I_LIM, -I_LIM, MF_GAUSS, I_WIGHT},
 .d_ruleN = {D_LIM, -D_LIM, MF_GAUSS, D_WIGHT},

 .o_rule = {O_LIM, -O_LIM, MF_NONE, 1.0f},
};


uint32_t fuzzyTest(void)
{
  uint32_t pass = 0;
  #if 1
  // Function Testing

  vLOG(vLOG_LEVEL_DEBUG, "deFuzzyifierCentroid Test ");
  
  //-----------------------------------------------------------------------------
  // PID Like Test
  //-----------------------------------------------------------------------------
  fuzzyInputs_s input = {
    .error = 0.0f,
    .errorLast = 0.0f,
    .errorDiff = 0.0f,
    .errorSum = 0.0f,
  };
  float fuzzyOutput = 0;
  // Test 4
  vLOG(vLOG_LEVEL_DEBUG, "Test 4 PID Test ");
  input.error     = -10.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error - 0.0f;
  input.errorSum  = -input.error - 0.0f;
  fuzzyOutput = FuzzyControllerPIDTest(&input, &fuzzyRulesS);
 // vLOG(vLOG_LEVEL_DEBUG, "Output: "FM"", FW(fuzzyOutput));
  

  // Test 5
  vLOG(vLOG_LEVEL_DEBUG, "Test 5 PID Test ");
  input.error     = -10.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error - 20.0f;
  input.errorSum  = -input.error + 0.0f;
  fuzzyOutput = FuzzyControllerPIDTest(&input, &fuzzyRulesS);
 // vLOG(vLOG_LEVEL_DEBUG, "Output: "FM"", FW(fuzzyOutput));
  
  
  // Test 6
  vLOG(vLOG_LEVEL_DEBUG, "Test 6 PID Test ");
  input.error     = -10.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error - -20.0f;
  input.errorSum  = -input.error - 0.0f;
  fuzzyOutput = FuzzyControllerPIDTest(&input, &fuzzyRulesS);
 // vLOG(vLOG_LEVEL_DEBUG, "Output: "FM"", FW(fuzzyOutput));
  

  #endif
  return pass;

}

