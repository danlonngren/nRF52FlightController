#include "FuzzyLogic.h"

#include <stdint.h>
#include <stdbool.h>
#include "common_math.h"

#include "nrf_port.h"

#include "FuzzyLogicMF.h"
#include "FuzzyLogicOperators.h"


///////////////////////////////////////////////////////////////////////////////

void fuzzyEvaluateMF(float input, rule_s * rule, float * outputWeighted, float *output)
{
  float temp = 0;
  
  ASSERT(outputWeighted != NULL);

  switch (rule->mfType)
  {
    case MF_LINEAR_POS:
      temp = linearMFPos(input, rule->highLim, rule->lowLim);
      break;
    case MF_LINEAR_NEG:
      temp = linearMFNeg(input, rule->highLim, rule->lowLim);
      break;     
    case MF_GAUSS:

      break;
    default: break;
  }

  // Return unweighted member value if requested
  if (output != NULL)
    *output = temp;

  *outputWeighted = temp * rule->weighting;
}


float FuzzyController(fuzzyInputs_s * inputs, fuzzyRules_s * constants) 
{
  // Process Inputs to useful signals ie. error, error difference, and error rate (PID)
 
  inputs->errorSum += inputs->error; 

  // Prevent integral windup
  if (inputs->errorSum >= constants->i_ruleP.highLim)
    inputs->errorSum = constants->i_ruleP.highLim;
  else if (inputs->errorSum <= constants->i_ruleP.lowLim)
    inputs->errorSum = constants->i_ruleP.lowLim;
 
  inputs->errorDiff = (inputs->errorLast - inputs->error);
  inputs->errorLast = inputs->error;

  //-----------------------------------------------------------------------------
  // Fuzzify Inputs
  //-----------------------------------------------------------------------------
  // Translate into fuzzy values ranging from 0 to 1;
  float weightedMembersP[3], weightedMembersN[3];
  float membersP[3], membersN[3];
  fuzzyEvaluateMF(inputs->error,     &constants->p_ruleP, &weightedMembersP[0], &membersP[0]); //linear posetive function (centre point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &constants->d_ruleP, &weightedMembersP[1], &membersP[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &constants->i_ruleP, &weightedMembersP[2], &membersP[2]);

  // Neg side
  fuzzyEvaluateMF(inputs->error,     &constants->p_ruleN, &weightedMembersN[0], &membersN[0]); //linear posetive function (centre point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &constants->d_ruleN, &weightedMembersN[1], &membersN[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &constants->i_ruleN, &weightedMembersN[2], &membersN[2]);
 
  //-----------------------------------------------------------------------------
  // Apply Rules
  //-----------------------------------------------------------------------------
  float fuzzyRules[6];
  fuzzyRules[0] = weightedMembersN[0]; //r1 - rule 1 and 2 acts as the Proportional part from fuzzyRules PID
  fuzzyRules[1] = fuzzyAND(weightedMembersN[0], weightedMembersP[1]); //r3 - rule 3 and 4 acts as the derivative part from fuzzyRules PID
  fuzzyRules[2] = fuzzyAND(weightedMembersN[0], weightedMembersN[2]); //r5 - rule 5 and 6 acts as the integral part from fuzzyRules PID

  fuzzyRules[3] = weightedMembersP[0]; //r2  
  fuzzyRules[4] = fuzzyAND(weightedMembersP[0], weightedMembersN[1]); //r4  
  fuzzyRules[5] = fuzzyAND(weightedMembersP[0], weightedMembersP[2]); //r6  

  //-----------------------------------------------------------------------------
  // Defuzzify
  //-----------------------------------------------------------------------------
  float posMember = 0; 
  float negMember = 0;
  for (uint32_t i = 0; i < 3; i++)
  {
    // Aggrigate Rules
    negMember += fuzzyRules[i];
    posMember += fuzzyRules[i + 3];
    
  }

  float fuzzySum = posMember + negMember;
  
  negMember *= constants->o_rule.lowLim;
  posMember *= constants->o_rule.highLim;
  
  return (posMember + negMember) / fuzzySum;
}


float FuzzyControllerPIDTest(fuzzyInputs_s * inputs, fuzzyRules_s * constants) 
{
  // Process Inputs to useful signals ie. error, error difference, and error rate (PID)
 
  inputs->errorSum += inputs->error; 

  // Prevent integral windup
  if (inputs->errorSum >= constants->i_ruleP.highLim)
    inputs->errorSum = constants->i_ruleP.highLim;
  else if (inputs->errorSum <= constants->i_ruleP.lowLim)
    inputs->errorSum = constants->i_ruleP.lowLim;
 
  inputs->errorDiff = (inputs->error - inputs->errorLast);
  inputs->errorLast = inputs->error;

  //-----------------------------------------------------------------------------
  // Fuzzify Inputs
  //-----------------------------------------------------------------------------
  // Translate into fuzzy values ranging from 0 to 1;
  float weightedMembersP[3], weightedMembersN[3];
  float membersP[3], membersN[3];
  fuzzyEvaluateMF(inputs->error,     &constants->p_ruleP, &weightedMembersP[0], &membersP[0]); //linear posetive function (centre point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &constants->d_ruleP, &weightedMembersP[1], &membersP[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &constants->i_ruleP, &weightedMembersP[2], &membersP[2]);

  // Neg side
  fuzzyEvaluateMF(inputs->error,     &constants->p_ruleN, &weightedMembersN[0], &membersN[0]); //linear posetive function (centre point 0.5 with output from 0 to 1)
  fuzzyEvaluateMF(inputs->errorDiff, &constants->d_ruleN, &weightedMembersN[1], &membersN[1]);
  fuzzyEvaluateMF(inputs->errorSum,  &constants->i_ruleN, &weightedMembersN[2], &membersN[2]);
 
  //-----------------------------------------------------------------------------
  // Apply Rules
  //-----------------------------------------------------------------------------
  float fuzzyRules[6];
  fuzzyRules[0] = weightedMembersN[0]; //r1 - rule 1 and 2 acts as the Proportional part from fuzzyRules PID
  fuzzyRules[1] = fuzzyOR(weightedMembersN[0], weightedMembersP[1]); //r3 - rule 3 and 4 acts as the derivative part from fuzzyRules PID
  fuzzyRules[2] = fuzzyOR(weightedMembersN[0], weightedMembersN[2]); //r5 - rule 5 and 6 acts as the integral part from fuzzyRules PID

  fuzzyRules[3] = weightedMembersP[0]; //r2  
  fuzzyRules[4] = fuzzyOR(weightedMembersP[0], weightedMembersN[1]); //r4  
  fuzzyRules[5] = fuzzyOR(weightedMembersP[0], weightedMembersP[2]); //r6  

  //-----------------------------------------------------------------------------
  // Defuzzify
  //-----------------------------------------------------------------------------
  float posMember = 0; 
  float negMember = 0;
  for (uint32_t i = 0; i < 3; i++)
  {
    // Aggrigate Rules
    negMember += weightedMembersN[i];
    posMember += weightedMembersP[i];
    
  }

  float fuzzySum = posMember + negMember;
  
  negMember *= constants->o_rule.lowLim;
  posMember *= constants->o_rule.highLim;
  
  return (posMember + negMember) / fuzzySum;
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
 .p_ruleP = {P_LIM, -P_LIM, MF_LINEAR_POS, P_WIGHT}, 
 .i_ruleP = {I_LIM, -I_LIM, MF_LINEAR_POS, I_WIGHT},
 .d_ruleP = {D_LIM, -D_LIM, MF_LINEAR_POS, D_WIGHT},

 .p_ruleN = {P_LIM, -P_LIM, MF_LINEAR_NEG, P_WIGHT}, 
 .i_ruleN = {I_LIM, -I_LIM, MF_LINEAR_NEG, I_WIGHT},
 .d_ruleN = {D_LIM, -D_LIM, MF_LINEAR_NEG, D_WIGHT},

 .o_rule = {O_LIM, -O_LIM, MF_NONE, 1.0f},
};


uint32_t fuzzyTest(void)
{
  uint32_t pass = 0;
  #if 1
  // Function Testing

  LOG("deFuzzyifierCentroid Test \r\n");
  
  // Test 1
  fuzzyInputs_s input;
  input.error     = 0.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error + 1.0f;//input.error - 5.0f;
  input.errorSum  = -input.error;
  float fuzzyOutput = FuzzyController(&input, &fuzzyRulesS);
  LOG("Test 1 \r\n" );
  LOG("Output: %i \r\n", (int32_t)(fuzzyOutput * 100));
  FLUSH();

  // Test 2
  input.error     = 1.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error;//input.error - 5.0f;
  input.errorSum  = -input.error + 0.0f;
  fuzzyOutput = FuzzyController(&input, &fuzzyRulesS);
  LOG("Test 2 \r\n");
  LOG("Output: %i \r\n", (int32_t)(fuzzyOutput * 100));
  FLUSH();

  // Test 3
  input.error     = 0.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error - 1.0f;//input.error - 5.0f;
  input.errorSum  = -input.error;
  fuzzyOutput = FuzzyController(&input, &fuzzyRulesS);
  LOG("Test 3 \r\n");
  LOG("Output: %i \r\n\n", (int32_t)(fuzzyOutput * 100));
  FLUSH();

  //-----------------------------------------------------------------------------
  // PID Like Test
  //-----------------------------------------------------------------------------

  // Test 4
  input.error     = 0.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error + 1.0f;//input.error - 5.0f;
  input.errorSum  = -input.error;
  fuzzyOutput = FuzzyControllerPIDTest(&input, &fuzzyRulesS);
  LOG("Test 4 PID Test \r\n");
  LOG("Output: %i \r\n", (int32_t)(fuzzyOutput * 100));
  FLUSH();

  // Test 5
  input.error     = 1.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error - 0.0f;//input.error - 5.0f;
  input.errorSum  = -input.error;
  fuzzyOutput = FuzzyControllerPIDTest(&input, &fuzzyRulesS);
  LOG("Test 5 PID Test \r\n");
  LOG("Output: %i \r\n", (int32_t)(fuzzyOutput * 100));
  FLUSH();
  
  // Test 6
  input.error     = 0.0f;
  input.errorDiff = 0.0f;
  input.errorLast = input.error - 1.0f;//input.error - 5.0f;
  input.errorSum  = -input.error;
  fuzzyOutput = FuzzyControllerPIDTest(&input, &fuzzyRulesS);
  LOG("Test 6 PID Test \r\n");
  LOG("Output: %i \r\n", (int32_t)(fuzzyOutput * 100));
  FLUSH();

  #endif
  return pass;

}

