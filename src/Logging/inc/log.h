#ifndef LOG_H
#define LOG_H

#include <stdint.h>

// Defines
#define LOG_VERSION       "v1.0.0" // Initial version
#define VLOG_DEBUG_LEVEL  vLOG_LEVEL_DEBUG
#define VLOG_MSG_MAX_CHAR 124

/**@brief Levels of debugging/logging information */
typedef enum {
  vLOG_LEVEL_OFF,     //!< No log info
  vLOG_LEVEL_WARNING, //!< WARNING  - Only warnings will be shown
  vLOG_LEVEL_ERROR,   //!< ERROR    - Only provide error messages and warnings
  vLOG_LEVEL_INFO,    //!< INFO     - Ignores all debug info and only logs
  vLOG_LEVEL_DEBUG,   //!< DEBUG    - Provides with all available logs and info
} eLogLevel;

/*Log structure*/
typedef struct {
  eLogLevel level;
  const uint8_t size;
  const uint8_t *pName;
} logModule_s;

/*Macro declaring variable and module*/
#define VLOG_MODULE_INIT(name, _level) \
static logModule_s vLogModule = {.pName = name, .size = sizeof(name), .level = _level};


uint32_t vLogInit(void);

_Bool vLogState(void);

void vLogOff(void);

void vLogOn(void);


/**
 * @brief Macro to be used in a formatted string to a pass float number to the log.
 */
#define FM "%1s%3d.%02d"


/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */
#define FW(val) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)


/**@brief Implementation details for NUM_VAR_ARGS */
#define NUM_VA_ASS_1_IMPL(                       \
    _ignored,                                          \
    _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10,       \
    _11, _12, _13, _14, _15, _16, _17, _18, _19, _20,  \
    _21, _22, _23, _24, _25, _26, _27, _28, _29, _30,  \
    _31, _32, _33, _34, _35, _36, _37, _38, _39, _40,  \
    _41, _42, _43, _44, _45, _46, _47, _48, _49, _50,  \
    _51, _52, _53, _54, _55, _56, _57, _58, _59, _60,  \
    _61, _62, N, ...) N

/**@brief Macro to get the number of arguments in a call variadic macro call.
 * First argument is not counted.
 *
 * param[in]    ...     List of arguments
 *
 * @retval  Number of variadic arguments in the argument list
 */
#define NUM_VA_ARGS1(...) NUM_VA_ASS_1_IMPL(__VA_ARGS__, 1, 1, 1,  \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                         \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                         \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                         \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                         \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                         \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, ~)



#define vLOG_FLOAT(type, fmt, num)  log_with_float(&vLogModule, type, __LINE__, fmt, num)
#define vLOG_NUM(type, fmt, num)  log_with_int(&vLogModule, type, __LINE__, fmt, num)


void log_with_float(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, float num);
void log_with_int(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, int num);

#define LOG(...) printf(__VA_ARGS__)
#define vLOG(type, ...) LOGTEST(type, __VA_ARGS__)

#define LOGX(N, ...)          CONCAT_2(LOG_, N) (__VA_ARGS__)
#define LOGTEST(type, ...)    LOGX(NUM_VA_ARGS1(__VA_ARGS__), type, __VA_ARGS__)

void log0(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg);
void logN(logModule_s *type, eLogLevel level, int32_t lineNum, const uint8_t *msg, ...);

#define LOG_0(type, fmt) \
        log0(&vLogModule, type, __LINE__, fmt)

#define LOG_1(type, fmt, ...) \
        logN(&vLogModule, type, __LINE__, fmt, __VA_ARGS__)


void utilPrintFloatArray(uint8_t *string, float *arr, uint8_t size);

void utilPrintXYZ(uint8_t *name, int32_t x, int32_t y, int32_t z);

void utilPrintFloat(uint8_t * string, float val);


#endif
