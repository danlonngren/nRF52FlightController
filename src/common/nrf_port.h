#ifndef NRF_PORT_H
#define NRF_PORT_H

#include "app_error.h"

#define ERROR_CHECK(ERR_CODE)                               \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
        {                                                   \
            APP_ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)


typedef uint32_t error_t;


/*-------------------------- Logging Functions --------------------------*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define LOG(...)     NRF_LOG_RAW_INFO( __VA_ARGS__)
#define FLUSH()      NRF_LOG_FLUSH()

void utilPrintFloatArray(uint8_t *string, float *arr, uint8_t size);

void utilPrintXYZ(uint8_t *name, int32_t x, int32_t y, int32_t z);


#endif