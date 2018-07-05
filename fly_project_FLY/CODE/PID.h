#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"
#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define IMU_UPDATE_FREQ   250		//250hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)		//积分常数


#define PID_PITCH_INTEGRATION_LIMIT   25.0 // 20.0
#define PID_ROLL_INTEGRATION_LIMIT    25.0 // 20.0
#define PID_YAW_INTEGRATION_LIMIT   20.0 // 20.0

#define Piddeadband   0	  //1~2
typedef struct
{
  float desired;     //< 被调量期望值
  float error;        //< 期望值-实际值
  float prevError;    //< 前一次偏差
  float integ;        //< 积分部分
  float deriv;        //< 微分部分
  float kp;           //< 比例参数
  float ki;           //< 积分参数
  float kd;           //< 微分参数
  float outP;         //< pid比例部分，调试用
  float outI;         //< pid积分部分，调试用
  float outD;         //< pid微分部分，调试用
  float iLimit;      //< 积分限制
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
