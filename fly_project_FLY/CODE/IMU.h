#ifndef __IMU_H
#define __IMU_H

#include "stm32f10x.h"

#include "USART.h"
#include "I2C.h"
#include "delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#include <math.h>
#define M_PI  (float)3.1415926535

//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * ypr); //更新姿态
float LPF_2nd(float newdata);//二阶低通算滤波

#endif

//------------------End of File----------------------------
