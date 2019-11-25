#ifndef TESTING_H_
#define TESTING_H_


#include <stdint.h>
#include "error_codes.h"

#define TEST_CASE(_test, _stringIn) test_case(_test, _stringIn, __FILE__, __LINE__)

void test_case(bool test, uint8_t stringIn[], uint8_t * pFile, int32_t lineNum);


#endif