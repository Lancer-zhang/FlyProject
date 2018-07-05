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

//Mini IMU AHRS �����API
void IMU_init(void); //��ʼ��
void IMU_getYawPitchRoll(float * ypr); //������̬
float LPF_2nd(float newdata);//���׵�ͨ���˲�

#endif

//------------------End of File----------------------------
