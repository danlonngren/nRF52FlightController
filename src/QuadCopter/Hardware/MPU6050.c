/*
 * MPU6050.c
 *
 * Created: 15/06/2019 14:38:25
 *  Author: Danlo
 */ 

#include "MPU6050.h"
#include "MPU6050_registers.h"

#include "board_config.h"
#include "nrf_port.h"

#include "nrf_drv_twi.h"

#include "twi.h"

static int16_t MPU6050_calibration[3]; 

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
//static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

#if 0
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}
#endif



error_t MPU6050_writeReg(uint8_t reg, uint8_t val)
{
    return twiWriteData(MPU6050_ADDRESS, reg, &val, sizeof(val));
}


static error_t MPU6050_readData(int16_t pOut[])
{
  uint8_t tx[14]; 

  error_t errCode;
  errCode = twiReadData(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, tx, sizeof(tx)); 

  uint32_t count = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    pOut[i] = (tx[count] << 8) | tx[count + 1];
    count += 2;
  }
  return errCode;
}

#define MPU6050_NUM_CALIBRATION 2000
error_t MPU6050_calibrate(void)
{
    error_t errCode = SUCCESS;
    int16_t arrData[7];
    for (uint16_t i = 0; i < MPU6050_NUM_CALIBRATION; i++)
    {
        errCode = MPU6050_readData(arrData);
        if (errCode != SUCCESS) break;

        MPU6050_calibration[0] += arrData[4];
        MPU6050_calibration[1] += arrData[5];
        MPU6050_calibration[2] += arrData[6];
    }
    MPU6050_calibration[0] /= MPU6050_NUM_CALIBRATION;
    MPU6050_calibration[1] /= MPU6050_NUM_CALIBRATION;
    MPU6050_calibration[2] /= MPU6050_NUM_CALIBRATION;	
    return errCode;
}

MPU6050_data_t MPU6050_getData(void)
{
    MPU6050_data_t data;
    int16_t arrData[7];
    data.errCode = MPU6050_readData(arrData);
    data.acc.x = arrData[0] - 98;
    data.acc.y = arrData[1] + 78;
    data.acc.z = arrData[2] + 20;
    data.gyro.x = ((float)arrData[4] - (float)MPU6050_calibration[0]) / 65.5;
    data.gyro.y = ((float)arrData[5] - (float)MPU6050_calibration[1]) / 65.5;
    data.gyro.z = ((float)arrData[6] - (float)MPU6050_calibration[2]) / 65.5;
    return data;
}


error_t MPU6050_init(void)
{
  error_t errCode;
  errCode = twiInit(SCL_PIN, SDA_PIN, 8);

  MPU6050_calibration[0] = 0;
  MPU6050_calibration[1] = 0;
  MPU6050_calibration[2] = 0;

  MPU6050_writeReg(MPU6050_RA_PWR_MGMT_1, 0x00);
  MPU6050_writeReg(MPU6050_RA_GYRO_CONFIG, (MPU6050_GYRO_FS_500 << 3));
  MPU6050_writeReg(MPU6050_RA_ACCEL_CONFIG, (MPU6050_ACCEL_FS_8 << 3));
  MPU6050_writeReg(MPU6050_RA_CONFIG, 0x03);
  
  MPU6050_calibrate();
  
  return errCode;
}


