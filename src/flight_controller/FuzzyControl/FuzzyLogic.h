#ifndef FUZZYLOGIC_H
#define FUZZYLOGIC_H

#include <stdint.h>


typedef enum {
  MF_NONE,
  MF_LINEAR_POS,
  MF_LINEAR_NEG,
  MF_GAUSS,
} mfType_e;

typedef struct {
  float error;
  float errorLast;
  float errorDiff;
  float errorSum;
} fuzzyInputs_s;

typedef struct {
  float     highLim;
  float     lowLim;
  mfType_e  mfType;
  float     weighting;
}rule_s; 

//typedef struct {
//  float   input;
//  rule_s  rule;
//} fuzzySet_s;

/**  */
typedef struct {
  rule_s      p_ruleP;
  rule_s      i_ruleP;
  rule_s      d_ruleP;

  rule_s      p_ruleN;
  rule_s      i_ruleN;
  rule_s      d_ruleN;

  rule_s      o_rule;  
} fuzzyRules_s;


uint32_t fuzzyTest(void);

float FuzzyController(fuzzyInputs_s * inputs, fuzzyRules_s * constants);





#endif 