#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"
#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define IMU_UPDATE_FREQ   250		//250hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)		//���ֳ���


#define PID_PITCH_INTEGRATION_LIMIT   25.0 // 20.0
#define PID_ROLL_INTEGRATION_LIMIT    25.0 // 20.0
#define PID_YAW_INTEGRATION_LIMIT   20.0 // 20.0

#define Piddeadband   0	  //1~2
typedef struct
{
  float desired;     //< ����������ֵ
  float error;        //< ����ֵ-ʵ��ֵ
  float prevError;    //< ǰһ��ƫ��
  float integ;        //< ���ֲ���
  float deriv;        //< ΢�ֲ���
  float kp;           //< ��������
  float ki;           //< ���ֲ���
  float kd;           //< ΢�ֲ���
  float outP;         //< pid�������֣�������
  float outI;         //< pid���ֲ��֣�������
  float outD;         //< pid΢�ֲ��֣�������
  float iLimit;      //< ��������
} pidsuite;



void pidInit(pidsuite* pid, const float desired, const float kp, const float ki, const float kd);
float pidUpdate(pidsuite* pid, const float new_measured,float expect,float gyro);
void pidSetIntegralLimit(pidsuite* pid, const float limit);
void PID_controllerInit(u8 *rxb);
void PID_CAL(float *YPR,float *motor,int *motor_tx,int thr);
int MOTORLimit(float value);
void Getdesireddata(int ch1,int ch2,int ch3,int ch4);
void controlmiddleinit(void);
float PID_core_control(pidsuite* pid,float outside_pid,float gyro);
//float yawcontrol(float gz,const float measured,float expect);

#endif
