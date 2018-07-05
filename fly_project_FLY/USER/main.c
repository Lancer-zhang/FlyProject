#include "stm32f10x.h"
#include "USART.h"
#include "I2C.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include "delay.h"
#include "IMU.h"
#include "pid.h"
#include "pwm.h"
#include "Timer.h"
#include "led.h"
#include "SPI_NRF.h"
#include <math.h>
u16 cnt_2ms;
u16 cnt_5ms;
u16 cnt_10ms;
u16 cnt_20ms;
float ypr[3]; // yaw pitch roll
float expect_roll=0;
float expect_pitch=0;
float expect_yaw=90;
float MOTO[4];
int MOTO_TX[4];
int Moto_Thr=1200;
extern float pid_roll;
extern float pid_pitch;
extern float pid_yaw;
u8 status_RT;
u8 status;	//用于判断接收/发送状态
u8 rxbuf[30];	 //发送缓冲
//float pitch_offset,roll_offset,yaw_offset,High_offset;
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))	//发送匿名地面站
#define KEY_Start 5

void Data_Send_Status(float Roll,float Pitch,float Yaw);//,int Moto_PWM_1,int Moto_PWM_2,int Moto_PWM_3,int Moto_PWM_4,int THROTTLE);

int main(void)
{	
  int pid_status=1;		 	
	SystemInit();
	delay_init(72);
	delay_ms(100);
  USART1_Config();
  delay_ms(100);
	Init_I2C();
	delay_ms(100);
	Init_I2C2();
  delay_ms(100);	
  SPI_NRF_Init(); 
	delay_ms(100);
	status=NRF_Check();
	/*判断连接状态*/  
  if(status != SUCCESS)	   
	 {
	   while(1);
	 } 
  TIM2_PWM_Init();
	delay_ms(100);	
	IMU_init();	 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//关闭JLINK， 
  LED_GPIO_Config();
		
	delay_ms(100);
	 TIM3_Config();
 cnt_2ms=0;
 cnt_5ms=0;
 cnt_10ms=0;
 cnt_20ms=0;	
delay_ms(1000);
delay_ms(1000);
delay_ms(1000);
	 while(1)
	{		
		GPIO_ResetBits(GPIOC,GPIO_Pin_12);
			NRF_RX_Mode();
		 status=NRF_Rx_Dat(rxbuf);
			status_RT=1;
	//	while(1){GPIO_ResetBits(GPIOB,GPIO_Pin_3);}	
		if(status==RX_DR&&rxbuf[0]==5)
		{
			 GPIO_ResetBits(GPIOD,GPIO_Pin_2);//no.2
			 Moto_Thr=((rxbuf[1]<<8)+rxbuf[2]);
       if(pid_status==1)
      {
	       PID_controllerInit(rxbuf); 
	       pid_status=0;
      }
			
      if(cnt_2ms>=2){
			IMU_getYawPitchRoll(ypr);
       
       cnt_2ms=0;			 }
			if(cnt_5ms>=5){
			 PID_CAL(ypr,MOTO,MOTO_TX,Moto_Thr);
			cnt_5ms=0;	
			}
			if(cnt_10ms>=10){
			 TIM2_Mode_Config(25000,MOTO[0]+rxbuf[21],MOTO[1]+rxbuf[22],MOTO[2]+rxbuf[23],MOTO[3]+rxbuf[24]);
			cnt_10ms=0;	
			}
       if(cnt_20ms>=10){
			//Data_Send_Status(pid_roll,pid_pitch,pid_yaw,MOTO_TX[0],MOTO_TX[1],MOTO_TX[2],MOTO_TX[3],Moto_Thr);
       Data_Send_Status(ypr[1],ypr[2],ypr[0]);//,MOTO_TX[0],MOTO_TX[1],MOTO_TX[2],MOTO_TX[3],Moto_Thr);
			 cnt_20ms=0;
			 }
		}
		else if(status==RX_DR&&rxbuf[0]==0)
		  {
			   TIM2_Mode_Config(25000,1000,1000,1000,1000);
				GPIO_SetBits(GPIOD,GPIO_Pin_2);//no.2		
				pid_status=1;
		  }	
	}
}


void Data_Send_Status(float Roll,float Pitch,float Yaw)//,int Moto_PWM_1,int Moto_PWM_2,int Moto_PWM_3,int Moto_PWM_4,int THROTTLE)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
  data_to_send[_cnt++]=0;
	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0-(int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	//串口发送数据
//	for(i=0;i<_cnt;i++)
	
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_1);//8
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_1);//9
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_2);//10
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_2);//11
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_3);//12
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_3);//13
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_4);//14
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_4);//15
//	data_to_send[_cnt++]=BYTE1(THROTTLE);//16
//	data_to_send[_cnt++]=BYTE0(THROTTLE);//17
//	data_to_send[1] = _cnt-4;
	//和校验
//	for(i=0;i<_cnt;i++)
//		sum+= data_to_send[i];
//	data_to_send[_cnt++]=sum;
	for(i=0;i<_cnt;i++)
	USART1_Putc(data_to_send[i]);
//if(status_RT==0)
	{
	//	NRF_TX_Mode();
	//	NRF_Tx_Dat(data_to_send);
	//	status_RT=1;
	}
	

}

