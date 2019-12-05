#ifndef LOG_H
#define LOG_H

#include <stdint.h>

#define VLOG_MSG_MAX_CHAR 124
#define VLOG_SYS_SIZE    13

/**@brief Contained Sub systems */
typedef enum {
  ELOG_SYS_SETUP = 0,     //!< Device Setup
  ELOG_SYS_CONTROL_LOOP,  //!< Main Control Loop
  ELOG_SYS_FUZZY_LOGIC,   //!< Fuzzy Logic controller
  ELOG_SYS_PID,           //!< PID Controller
  ELOG_SYS_IMU,           //!< Sensor IMU Fusion
  ELOG_SYS_MOTORS,        //!< Motor Driver
  ELOG_SYS_RECEIVER,      //!< Receiver Driver
  ELOG_SYS_SENSORS,       //!< Sensor Drivers
  ELOG_SYS_TIMER,         //!< Timer Driver
  ELOG_SYS_SPI,           //!< SPI Driver
  ELOG_SYS_TWI,           //!< TWI Driver
  ELOG_SYS_BLE,           //!< BLE
} eLogSubSystem;

/**@brief Levels of debugging/logging information */
typedef enum {
  ELOG_LEVEL_OFF,     //!< No log info
  ELOG_LEVEL_WARNING, //!< WARNING  - Only warnings will be shown
  ELOG_LEVEL_ERROR,   //!< ERROR    - Only provide error messages and warnings
  ELOG_LEVEL_INFO,    //!< INFO     - Ignores all debug info and only logs
  ELOG_LEVEL_DEBUG,   //!< DEBUG    - Provides with all available logs and info
} eLogLevel;

typedef int (*printfFncPointer_t)(const char *, ...); // Printf function pointer. Can be changed to anything

/**@brief Structure for initializing and storing logging data. */
typedef struct {
  _Bool  state; //!< Logging state
  const printfFncPointer_t pFncPrintf; //!< Pointer to printing function
  eLogLevel subSysLevel[VLOG_SYS_SIZE]; //!< Syb Systems logging levels
  const uint8_t pSrcDirName[];           //!< Project directory file name

}sLogSetup;

uint32_t vLogInit(sLogSetup * logInit);

void vLogOff(void);

void vLogOn(void);

void vLogSetLevel(eLogLevel level, eLogSubSystem sys);

#define VLOG(sys, level, msg) vLog(sys, level, msg, __FILE__, __LINE__);
void vLog(eLogSubSystem sys, eLogLevel level, uint8_t * msg, uint8_t *pFile, uint32_t lineNumber);

#define VLOG_WITH_NUM(sys, level, msg, num) vLogWithNum(sys, level, msg, num, __FILE__, __LINE__);
void vLogWithNum(eLogSubSystem sys, eLogLevel level, uint8_t * msg, uint32_t number, uint8_t *pFile, uint32_t lineNumber);

#define VLOG_WITH_FLOAT(sys, level, msg, num) vLogWithFloat(sys, level, msg, num,__FILE__, __LINE__);
void vLogWithFloat(eLogSubSystem sys, eLogLevel level, uint8_t * msg, float number, uint8_t *pFile, uint32_t lineNumber);

#endif
