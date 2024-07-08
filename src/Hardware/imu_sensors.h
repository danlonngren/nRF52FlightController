/*
 * MPU60050h.h
 *
 * Created: 15/06/2019 14:38:49
 *  Author: Danlo
 */ 

#ifndef IMU_SENSOR_H_
#define IMU_SENSOR_H_

#include <stdint.h>
#include "error_codes.h"
#include "vector_math.h"

#include "registers.h"

error_t imu_sensors_MPU6050_init(void);

error_t imu_sensors_mpuGyroCalibration(uint32_t maxSamples);

error_t imu_sensors_imuSensorsMagHardSoftCalibration(float *dest1, float *dest2, uint32_t numberSamples);

/**@brief Returns 9DOF data from MPU9050
 * Gyro Data is in Radians per seconds and offset, calibration has been subtracted.
 * Acc Data is in terms of g and offset has been subtracted.
 */
error_t imu_sensors_GetData(vector3_t *gyro, vector3_t *acc, vector3_t *mag);

/**@brief Gets the corrected gyro rate in radians per second */
error_t imu_sensors_GetMag(vector3_t *gyro);

/**@brief Gets the corrected accelerometer in g */
error_t imu_sensors_GetAccel(vector3_t *acc);

/**@brief Gets the corrected mag in mG [+-400] */
error_t imu_sensors_GetMag(vector3_t *mag);

#endif /* MPU60050H_H_ */