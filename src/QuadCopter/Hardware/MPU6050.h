/*
 * MPU60050h.h
 *
 * Created: 15/06/2019 14:38:49
 *  Author: Danlo
 */ 


#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDRESS 0x68
#define MPU6050_W_ADDRESS ((0x68 << 1) + I2C_WRITE)
#define MPU6050_R_ADDRESS ((0x68 << 1) + I2C_READ)

#include <stdint.h>
#include "error_codes.h"

typedef struct
{
    float x;
    float y;
    float z;	
}xyz_t;
 
typedef struct  
{
    xyz_t gyro;
    xyz_t acc;
    error_t errCode;
}MPU6050_data_t;

error_t MPU6050_init(void);

MPU6050_data_t MPU6050_getData(void);


#endif /* MPU60050H_H_ */