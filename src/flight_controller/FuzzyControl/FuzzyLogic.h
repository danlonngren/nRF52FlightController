#ifndef FUZZYLOGIC_H
#define FUZZYLOGIC_H

#include <stdint.h>


#define FUZZY_TYPE 1

#define FL_OUTPUT       100.0f
#define FL_OUTPUT_POS  FL_OUTPUT
#define FL_OUTPUT_NEG   (-FL_OUTPUT)


typedef enum {
  OUTPUT_NONE,
  OUTPUT_NEG,
  OUTPUT_POS,
} eOutputType;

typedef enum {
  MF_NONE,
  MF_LINEAR_POS,
  MF_LINEAR_NEG,
  MF_GAUSS,
} eMFType;

typedef enum {
  OPERATOR_NONE,
  OPERATOR_NOT,
  OPERATOR_AND,
  OPERATOR_OR,
  OPERATOR_PAND,
  OPERATOR_POR,
} eFuzzyOperator;

typedef struct {
  float error;
  float errorLast;
  float errorDiff;
  float errorSum;
} fuzzyInputs_s;

typedef struct {
  // Membership function data
  float     highLim;
  float     lowLim;
  eMFType  mfType;
  // Input Associated
  float input;

  float fuzzyResult;
} rule_s; 


//typedef struct {
//
//  sFuzzyRule * fuzzyRules[10];
//
//} sFuzzyInterface;

typedef struct {
  // Membership function data
  float     highLim;
  float     lowLim;
  eMFType  mfType;
  // Input Associated
  float input;

  float fuzzyResult;
} sFuzzySet; 

int32_t fuzzyAddSet(sFuzzySet *pSet, eMFType mf, float highLim, float lowLim); 

typedef struct {
  rule_s *pRule1;
  rule_s *pRule2;
  eFuzzyOperator opType;
  eOutputType outType;
  float setResult;
} sFuzzyRule;

typedef struct {
  float a;
  float b;
  float c;
} sFuzzySugenoConst;

float fuzzySugenoCalcOut(float *in1, float *in2, sFuzzySugenoConst c);



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