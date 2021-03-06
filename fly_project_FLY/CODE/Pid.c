#include "pid.h"
#include "math.h"
#include "IMU.h"
pidsuite pidRoll;					   //横滚角pid
pidsuite pidPitch;					   //俯仰角pid
pidsuite pidYaw;
pidsuite pid_core_roll;
pidsuite pid_core_pitch;
pidsuite pid_core_yaw;
//int Motor_Thr=0;					   //油门
//int Motor_Ele=0;					   //俯仰期望
//int Motor_Ail=0;					   //横滚期望
//int Motor_Rud=0;					   //航向期望
float mygetqval[9];
float pid_roll;
float pid_pitch;
float pid_yaw;
float PID_ROLL_KP = 0.0;//0.4//0.46
float PID_ROLL_KI = 0.0;//0//0.0 //2.0	 0.1
float PID_ROLL_KD = 0.0;//0.15//0.1
//0.2	0.0 0.02
//0.18	0.0	0.025
float test;

float PID_PITCH_KP = 0.0;//0.4 
float PID_PITCH_KI = 0.0;//0.0		  0.1
float PID_PITCH_KD = 0.0;//0.09


float PID_YAW_KP  = 0.0;					  //0.5~1.0			 2大了
float PID_YAW_KI = 0.0;
float  PID_YAW_KD = 0.0;					   //参数前加负�

float pitch_croe_KP = 0;//0.4 
float pitch_croe_KI = 0.0;//0.0		  0.1
float pitch_croe_KD =0;

float roll_croe_KP = 0;//0.4 
float roll_croe_KI = 0.0;//0.0		  0.1
float roll_croe_KD =0;

float yaw_croe_KP = 0;//0.4 
float yaw_croe_KI = 0;//0.0		  0.1
float yaw_croe_KD =0;
/*------------------------------------------pid结构初始化-----------------------------------------*/
//输入参数：结构体指针，期望值，kp,ki,kd
void pidInit(pidsuite* pid, const float desired, const float kp,
             const float ki, const float kd)
{

  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}

/*----------------------------------------------pid输出更新------------------------------------------*/
//输入参数：pid结构体指针，测量值 ,期望值=0(自稳)
//输出：pid输出
float pidUpdate(pidsuite* pid, const float new_measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;

  pid->desired=expect;			 				//获取期望角度，期望值为0

  //pid->error = new_measured-pid->last_measured;	 	  //偏差：期望-测量值
  pid->error = pid->desired - new_measured;	 	  //偏差：期望-测量值
  pid->integ += pid->error * IMU_UPDATE_DT;	  //偏差积分
  
  if (pid->integ > pid->iLimit)				  //作积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }				 
  //gyro=(pid->error - pid->prevError) / IMU_UPDATE_DT;	
  //pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//微分	 应该可用陀螺仪角速度代替
  pid->deriv = -gyro;
  if(fabs(pid->error)>Piddeadband)									//pid死区
  {
		  pid->outP = pid->kp * pid->error;								 //方便独立观察
		  pid->outI = pid->ki * pid->integ;
		  pid->outD = pid->kd * pid->deriv;
		
		  output = (pid->kp * pid->error) +
		           (pid->ki * pid->integ) +
		           (pid->kd * pid->deriv);
  }
  else
  {
  		  output=lastoutput;
  }
 // pid->last_measured=new_measured;
  pid->prevError = pid->error;							 		//更新前一次偏差
  lastoutput=output;

  return output;
}

/*----------------------------------------------pid积分部分限制值-------------------------------------------*/
void pidSetIntegralLimit(pidsuite* pid, const float limit)
{
  pid->iLimit = limit;
}
float PID_core_control(pidsuite* pid,float outside_pid,float gyro)
{
	//pidUpdate(
	float gyro_old=0,pidcore_out;
	pid->error=outside_pid - 3.5*gyro;
	 pidcore_out = pid->kp * pid->error + pid->kd * (pid->error - pid->prevError);
	pid->prevError = pid->error;	
	return pidcore_out;
}

void PID_controllerInit(u8 *rxb)
{
  PID_ROLL_KP =((float)((rxb[3]<<8)+rxb[4]))/1000;
	PID_ROLL_KI =((float)((rxb[5]<<8)+rxb[6]))/1000;
	PID_ROLL_KD =((float)((rxb[7]<<8)+rxb[8]))/1000;
	PID_PITCH_KP =((float)((rxb[9]<<8)+rxb[10]))/1000;
	PID_PITCH_KI =((float)((rxb[11]<<8)+rxb[12]))/1000;
	PID_PITCH_KD =((float)((rxb[13]<<8)+rxb[14]))/1000;
	PID_YAW_KP =((float)((rxb[15]<<8)+rxb[16]))/1000;
	PID_YAW_KI =((float)((rxb[17]<<8)+rxb[18]))/1000;
	PID_YAW_KD =((float)((rxb[19]<<8)+rxb[20]))/1000; 

	roll_croe_KP =((float)(rxb[25]))/1000;
	roll_croe_KD =((float)(rxb[26]))/1000;
	pitch_croe_KP =((float)(rxb[27]))/1000;
	pitch_croe_KD =((float)(rxb[28]))/1000;
	yaw_croe_KP =((float)(rxb[29]))/1000;
	yaw_croe_KD =((float)(rxb[30]))/1000;
	
	pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
	pidInit(&pidYaw,0,PID_YAW_KP,PID_YAW_KI,PID_YAW_KD);
	
  pidInit(&pid_core_roll, 0, roll_croe_KP, roll_croe_KI, roll_croe_KD);
  pidInit(&pid_core_pitch, 0, pitch_croe_KP, pitch_croe_KI, pitch_croe_KD);
	pidInit(&pid_core_yaw,0,yaw_croe_KP,yaw_croe_KI,yaw_croe_KD);
	pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);
}


//------------------------------------pid控制------------------------------------
void PID_CAL(float *YPR,float *motor,int *motor_tx,int thr)		  //PID参数计算
{

      pid_roll = pidUpdate(&pidRoll,YPR[1],0,mygetqval[3]);
	    pid_pitch = pidUpdate(&pidPitch,YPR[2],0,mygetqval[4]);
     	pid_yaw = pidUpdate(&pidYaw,YPR[0],0,mygetqval[5]);
	    
	    pid_roll = PID_core_control(&pid_core_roll,pid_roll,mygetqval[3]);
	    pid_pitch = PID_core_control(&pid_core_pitch,pid_pitch,mygetqval[4]);
	    pid_yaw = PID_core_control(&pid_core_yaw,pid_yaw,mygetqval[5]);
	
	 //  thr = thr/cos(YPR[1]/57.3)/cos(YPR[2]/57.3);
	   	motor[0]=MOTORLimit(thr-300-pid_pitch-(pid_roll));//-pid_yaw));	 			//0号电机-(pid_roll)
      motor[1]=MOTORLimit(thr-300-pid_pitch+(pid_roll));//-pid_yaw));				//1号电机+(pid_roll)

	motor[2]=MOTORLimit(thr -300+pid_pitch+(pid_roll));//+pid_yaw));				//2号电机+(pid_roll)
      motor[3]=MOTORLimit(thr -300+pid_pitch-(pid_roll));//+pi               -pid_roll
  
	
	    motor_tx[0]=motor[0]/250;
			motor_tx[1]=motor[1]/250;
			motor_tx[2]=motor[2]/250;
			motor_tx[3]=motor[3]/250;
			  
	if(thr<=1024)
	{
		motor[0]=1020;
		motor[1]=1020;
		motor[2]=1020;
		motor[3]=1020;
	}		  
}

//-----------------------------------------电机输出限制-------------------------------------------
int MOTORLimit(float value)
{
  	  if(value>2500)
	    {
		  	value=2400;
		}
	  else if(value<1025)
		{
			value=1025;
		}
	  else 
		{
		   value=value;
		}

	  return value;
}	
/*
void Getdesireddata(int ch1,int ch2,int ch3,int ch4)
{

		if(ch1<250){	ch1=250;}			   //油门限制
		if(ch1>420){	ch1=420;}	

		if(ch2<250){	ch2=250;}
		if(ch2>500){	ch2=500;}

		if(ch3<250){	ch3=250;}
		if(ch3>500){	ch3=500;}

		if(ch4<250){	ch4=250;}
		if(ch4>500){	ch4=500;}  


		Motor_Thr=ch1;					   			   //油门期望TIM5ch1-A0-遥控器通道3
		Motor_Ele=(int)(ch2-Elemiddle);					   //俯仰期望TIM5ch2-A1-遥控器通道2		 值域-100~100
	    Motor_Ail=(int)(ch3-Ailmiddle);					   //横滚期望TIM5ch3-A2-遥控器通道4
		Motor_Rud=(int)(ch4-Rudmiddle);					   //航向期望TIM5ch4-A3-遥控器通道1

		Motor_Ele=Motor_Ele/5;				//转化单位为角度-31~31度
		Motor_Ail=Motor_Ail/5;

		if(Motor_Rud<4&&Motor_Rud>-4)
		{
			  Motor_Rud=0;
		}
		else
		{Motor_Rud=Motor_Rud/2;}				//-125~125度
				
}

float yawcontrol(float gz,const float measured,float expect)			//角速度，测量，期望
{
		  float output;
//		  static float lastoutput=0;
		  float yaw_error=0;

		  yaw_error = expect - measured;	 	  //偏差：期望-测量值
		
		  output=yaw_kp*yaw_error+yaw_kd*gz;
//		  lastoutput=output;	
		  return output;	
}	 
*/

/*void controlmiddleinit(void)
{
		unsigned char i;
		for(i=0;i<10;i++)
		{
			Elemiddle+=pwmout3;	
			delay_ms(30);
		}
		Elemiddle=Elemiddle/10;	

		for(i=0;i<10;i++)
		{
			Ailmiddle+=pwmout4;	
			delay_ms(25);
		}
		Ailmiddle=Ailmiddle/10;	

		for(i=0;i<10;i++)
		{
			Rudmiddle+=pwmout2;	
			delay_ms(25);
		}
		Rudmiddle=Rudmiddle/10;	
} 	 */

