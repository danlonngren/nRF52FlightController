/*
 * MPU6050.c
 *
 * Created: 15/06/2019 14:38:25
 *  Author: Danlo
 */ 

#include "imu_sensors.h"

#include "nrf_delay.h"
#include "twi.h"

#include "board_config.h"
#include "nrf_port.h"

#include "common_math.h"
#include "vector_math.h"
#include "filters.h"

#include "parameters.h"


#define BIAS_DEFAULT_GYRO   {0, 0, 0}
#define BIAS_DEFAULT_ACC    {537.0f,  305.0f,  -443.0f}
#define BIAS_DEFAULT_MAG    {3.59619f, 374.32f, -306.157f}

#define IMU_MAG_SCALAR_DEFAULT  {1.02349f, 1.0391f, 0.942883f}

#define IMU_GYRO_RES_500   0.015267175572f // 1 / 65.5
#define IMU_ACC_RES_8      0.000244140625f // 1 / 4096
#define IMU_MAG_RES_14     1.499389499f // 10.*4912./32760.0

static float gyroBiasMeasured[3]  = BIAS_DEFAULT_GYRO;
static float accBiasMeasured[3]   = BIAS_DEFAULT_ACC;
static float magBiasMeasured[3]   = BIAS_DEFAULT_MAG;
static float magScalar[3]         = IMU_MAG_SCALAR_DEFAULT;
static float magASA[3]            = {1.0f, 1.0f, 1.0f};

static const float imuGRes  = IMU_GYRO_RES_500;  // 1 / 65.5
static const float imuARes  = IMU_ACC_RES_8; // 1 / 4096
static const float imuMRes  = IMU_MAG_RES_14;  // 10.*4912./32760.0


error_t mpuReadReg(uint8_t address, uint8_t reg, uint8_t *val)
{
    return twiReadData(address, reg, val, sizeof(val));
}

error_t mpuWriteReg(uint8_t address, uint8_t reg, uint8_t val)
{
    return twiWriteData(address, reg, &val, sizeof(val));
}


static error_t mpuReadAxisData(uint8_t address, uint8_t reg, int16_t *pOut, bool HorL)
{
  uint8_t tx[7];
 
  error_t errCode = twiReadData(address, reg, tx, sizeof(tx)); 

  uint8_t indexOut = 0, byteCount = 0;
  for (indexOut = 0; indexOut < 3; indexOut++)
  {
    if (HorL == true)
    {
        pOut[indexOut] = uint8toInt16(tx[byteCount], tx[byteCount + 1]);
    }
    else
    {
        pOut[indexOut] = uint8toInt16(tx[byteCount + 1], tx[byteCount]);
    }
    byteCount += 2;
  }
  return errCode;
}


error_t mpuReadAccXYZ(int16_t *out)
{

    uint8_t tx[6];
    error_t errCode = twiReadData(MPU_ADDRESS, MPU_A_XOUT_HSTART, tx, sizeof(tx)); 
    out[0] = uint8toInt16(tx[0], tx[1]);
    out[1] = uint8toInt16(tx[2], tx[3]);
    out[2] = uint8toInt16(tx[4], tx[5]);
    return errCode;
}

error_t mpuReadGyroXYZ(int16_t *out)
{
    uint8_t tx[6];
    error_t errCode = twiReadData(MPU_ADDRESS, MPU_G_XOUT_HSTART, tx, sizeof(tx)); 
    out[0] = uint8toInt16(tx[0], tx[1]);
    out[1] = uint8toInt16(tx[2], tx[3]);
    out[2] = uint8toInt16(tx[4], tx[5]);
    return errCode;
}


error_t mpuReadMagXYZ(int16_t *out)
{
    error_t errCode;

    uint8_t tx[8];
    uint8_t st1 = 0;
    while ((st1 & 0x01))
    {
        twiReadData(MAG_ADDRESS, 0x02, &st1, 1);
    }
    errCode = twiReadData(MAG_ADDRESS, MAG_XOUT_LSTART, tx, sizeof(tx)); 
    out[0] = (uint8toInt16(tx[1], tx[0]));
    out[1] = (uint8toInt16(tx[3], tx[2]));
    out[2] = (uint8toInt16(tx[5], tx[4]));
    return errCode;
}


error_t imuSensorGetGyro(vector3_t *gyro)
{
    int16_t dataOut[3]; // Temp variable for reading IMU data
    // Get gyro Data and Convert to Rads per second
    error_t retCode = mpuReadGyroXYZ(dataOut);
    gyro->v[0] = (((float)dataOut[0] - gyroBiasMeasured[0])) * imuGRes;
    gyro->v[1] = (((float)dataOut[1] - gyroBiasMeasured[1])) * imuGRes;
    gyro->v[2] = (((float)dataOut[2] - gyroBiasMeasured[2])) * imuGRes;
    return retCode;
}

error_t imuSensorGetAcc(vector3_t *acc)
{
    int16_t dataOut[3]; // Temp variable for reading IMU data
    // Read Accelerometer         
    error_t retCode = mpuReadAccXYZ(dataOut);
    acc->v[0] = ((float)dataOut[0] - accBiasMeasured[0])* imuARes;
    acc->v[1] = ((float)dataOut[1] - accBiasMeasured[1])* imuARes;
    acc->v[2] = ((float)dataOut[2] - accBiasMeasured[2])* imuARes;
    return retCode;
}

error_t imuSensorGetMag(vector3_t *mag)
{
    int16_t dataOut[3]; // Temp variable for reading IMU data
    error_t retCode = mpuReadMagXYZ(dataOut);
    mag->v[0] = (float)dataOut[0] * imuMRes * magScalar[0] * magASA[0] - magBiasMeasured[0];
    mag->v[1] = (float)dataOut[1] * imuMRes * magScalar[1] * magASA[1] - magBiasMeasured[1];
    mag->v[2] = (float)dataOut[2] * imuMRes * magScalar[2] * magASA[2] - magBiasMeasured[2];
    return retCode;
}

error_t imuSensorGetData(vector3_t *gyro, vector3_t *acc, vector3_t *mag)
{
    static vector3_t vGyroRaw;
    static vector3_t vAccRaw;
    static vector3_t vMagRaw;
    
    // Read Accelerometer         
    error_t retCode = imuSensorGetAcc(&vAccRaw);
    if (retCode != 0) return retCode;
    // Get gyro Data and Convert to Rads per second
    retCode = imuSensorGetGyro(&vGyroRaw);
    if (retCode != 0) return retCode;

    retCode = imuSensorGetMag(mag);
    if (retCode != 0) return retCode;

    for (uint8_t axis = 0; axis < 3; axis++)
    {
        gyro->v[axis] = filterSinglePoleLowPass(&gyro->v[axis], &vGyroRaw.v[axis], FILTER_COMP_GAIN_GYRO_RATE);
        acc->v[axis]  = filterSinglePoleLowPass(&acc->v[axis], &vAccRaw.v[axis],  FILTER_COMP_GAIN_GYRO_RATE);
       // mag->v[axis]  = filterSinglePoleLowPass(&mag->v[axis], &vMagRaw.v[axis],  FILTER_COMP_GAIN_GYRO_RATE);
    }

    return retCode;   
}


error_t imuSensorsMagHardSoftCalibration(float *dest1, float *dest2, uint32_t numberSamples)
{
    float mag_max[3] = {(float)INT16_MIN, (float)INT16_MIN, (float)INT16_MIN};
    float mag_min[3] = {(float)INT16_MAX, (float)INT16_MAX, (float)INT16_MAX};
    
    float mag_bias[3];
    float mag_scale[3];
    
    LOG("Mag Calibration Started \r\n");
    FLUSH();
    int16_t samples = 0;
    while (samples < numberSamples)
    {
        int16_t mag[3];
        mpuReadMagXYZ(mag);
    
        for (uint8_t x= 0; x < 3; x++)
        {
            if (mag[x] > mag_max[x]) mag_max[x] = mag[x];
            if (mag[x] < mag_min[x]) mag_min[x] = mag[x];
        }   
        nrf_delay_ms(10);
        samples++;
    }
    
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*imuMRes*magASA[0];  // save mag biases in  for main program
    dest1[1] = (float) mag_bias[1]*imuMRes*magASA[1];   
    dest1[2] = (float) mag_bias[2]*imuMRes*magASA[2];  
    
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

    utilPrintFloatArray("Hard Bias: ", dest1, 3);
    utilPrintFloatArray("Soft Bias: ", dest2, 3);

}



error_t mpuGyroCalibration(uint32_t maxSamples)
{
    error_t errCode = SUCCESS;
    int16_t arrData[3];
    for (uint16_t i = 0; i < maxSamples; i++)
    {
        uint8_t dataReady = 0;
        int32_t timeout = 5000;
        while ((dataReady & 0x01) == 0) // Wait for data ready
        {
          errCode = mpuReadReg(MPU_ADDRESS, MPU_INT_STATUS, &dataReady);
          if (errCode != SUCCESS || timeout <= 0)
          {
              return errCode;
          }

          timeout--;
        }

        errCode = mpuReadAxisData(MPU_ADDRESS, MPU_G_XOUT_HSTART, arrData, true);
        if (errCode != SUCCESS) 
          break;


        gyroBiasMeasured[0] += arrData[0];
        gyroBiasMeasured[1] += arrData[1];
        gyroBiasMeasured[2] += arrData[2];
        nrf_delay_ms(3);
    }
    gyroBiasMeasured[0] /= (float)maxSamples;
    gyroBiasMeasured[1] /= (float)maxSamples;
    gyroBiasMeasured[2] /= (float)maxSamples;	
    return errCode;
}




error_t MPU6050_init(void)
{
  error_t errCode = SUCCESS;
  
  errCode = mpuWriteReg(MPU_ADDRESS, MPU_PWR_MGMT_1, 0x00);
  if (errCode != 0) return errCode;
  errCode = mpuWriteReg(MPU_ADDRESS, MPU_SMPLRT_DIV, 0x00);
  if (errCode != 0) return errCode;
  errCode = mpuWriteReg(MPU_ADDRESS, MPU_GYRO_CONFIG, (MPU6050_GYRO_FS_500 << 3));
  if (errCode != 0) return errCode;
  errCode = mpuWriteReg(MPU_ADDRESS, MPU_ACCEL_CONFIG, (MPU6050_ACCEL_FS_8 << 3));
  if (errCode != 0) return errCode;
  errCode = mpuWriteReg(MPU_ADDRESS, MPU_INT_PIN_CFG, 0x22);
  if (errCode != 0) return errCode;
  errCode = mpuWriteReg(MPU_ADDRESS, MPU_CONFIG, 0x03);
  if (errCode != 0) return errCode;
  errCode = mpuWriteReg(MAG_ADDRESS, MPU_MAG_CNTL, 0b00001111);
  if (errCode != 0) return errCode;
  
  // Read magnetometer sensitivity
  uint8_t val = 0;
  errCode = mpuReadReg(MAG_ADDRESS, MPU_MAG_ASAX, &val);
  magASA[0] = ((((float)val-128.0f)*0.5f)/128.0f)+1.0f;
  if (errCode != 0) return errCode;

  errCode = mpuReadReg(MAG_ADDRESS, MPU_MAG_ASAY, &val);
  magASA[1] = ((((float)val-128.0f)*0.5f)/128.0f)+1.0f;
  if (errCode != 0) return errCode;

  errCode = mpuReadReg(MAG_ADDRESS, MPU_MAG_ASAZ, &val);
  magASA[2] = ((((float)val-128.0f)*0.5f)/128.0f)+1.0f;
  if (errCode != 0) return errCode;

  errCode = mpuWriteReg(MAG_ADDRESS, MPU_MAG_CNTL, 0x16);
  
  errCode = mpuReadReg(MPU_ADDRESS, MPU_ACCEL_CONFIG, &val);
  LOG("Acc Config Register: 0x%x \r\n", val);
  errCode = mpuReadReg(MPU_ADDRESS, MPU_GYRO_CONFIG, &val);
  LOG("Acc Config Register: 0x%x \r\n", val);


  return errCode;
}


