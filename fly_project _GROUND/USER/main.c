#include "stm32f10x.h"
#include "USART.h"
#include "delay.h"
#include "key.h"
#include "led.h"
#include "SPI_NRF.h"
#include "ADC.h"
#include <math.h>
__IO uint16_t ADC_ConvertedValue;
u8 status;	//用于判断接收/发送状态
u8 txbuf[32];	 //发送缓冲
u8 rxbuf[20];			 //接收缓冲
int i=0;
int THr;
u8 status_TR;
u8 thr[4];
typedef struct PID{float P,I,D;}PID;
PID PID_ROL,PID_PIT,PID_YAW,core_roll,core_pitch,core_yaw;
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))	//发送匿名地面站
#define KEY_Start 5

void Delay2(u32 n);
void send_station(void);
void set_pid(u8 *buf);
extern u16 Time2;

int main(void)
{
	u8 LED;//LED状态
	u8 key;//连续按下状态
  status_TR=1;
  /* 串口1初始化 */
	USART1_Config();
	KEY_GPIO_Config();
  ADC_Config();
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//关闭JLINK， 
  LED_GPIO_Config();
	SPI_NRF_Init();
	/*初始化外部中断函数*/
	//EXTI_Config();
	 /*检测NRF模块与MCU的连接*/
   //	status = NRF_Check(); 
status=NRF_Check();
	/*判断连接状态*/  
   if(status != SUCCESS)	 while(1);

while(1)
{
	LED1_ON;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0)
 		{
	 		Delay2(10);
	 		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0)
	 		{
					if(key==1)
					{
						Delay2(300);
						key=0;									
					}else
					{			 
				  		LED = GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_2);
						if(LED ==1)
						{
							LED2_ON;
						  txbuf[0]=5;	
							
						}  
						else 
						{
						  LED2_OFF;
						  txbuf[0]=0;

						}
						key=1;
					}
			
	 		}

 		}		
		
	 THr=((float)ADC_ConvertedValue/3290*1400+800);
		if(THr>1620)THr=1620;
		txbuf[1]=BYTE1(THr);
  	txbuf[2]=BYTE0(THr);
		set_pid(txbuf);

		{
			NRF_TX_Mode();
		  NRF_Tx_Dat(txbuf);
  //    status_TR=0;			
		}			
/*	
if(txbuf[0]==5)
{
	NRF_RX_Mode();	 
	NRF_Rx_Dat(rxbuf);
	send_station();
	status_TR=1;	
}*/
//else LED1_OFF;

}

	

}

void Delay2(u32 n)
{	u32 i,j;
	for(j=0;j<n;j++)
	for(i=0;i<10000;i++);
	  
}
void send_station()
{
	 u8	send_status[20];
	u8 send_pwm[20];
	
	if(rxbuf[0]==0xAA)
		{
			int k;
			unsigned char cnt_s=0,cnt_p=0,sum1=0,sum2=0;
			send_status[cnt_s++]=0xAA;
			send_status[cnt_s++]=0xAA;
			send_status[cnt_s++]=0x01;
			send_status[cnt_s++]=6;
			for(k=0;k<6;k++)
				send_status[cnt_s++]=rxbuf[k+2];
      for(i=0;i<cnt_s;i++)
		    sum1+= send_status[i];
	    send_status[cnt_s++]=sum1;
for(i=0;i<cnt_s;i++)
		USART1_Putc(send_status[i]);
			send_pwm[cnt_p++]=0xAA;
			send_pwm[cnt_p++]=0xAA;
			send_pwm[cnt_p++]=0x06;
			send_pwm[cnt_p++]=8;
			for(k=0;k<8;k++)		
				send_pwm[cnt_p++]=rxbuf[k+8];
			for(i=0;i<cnt_p;i++)
		    sum2+= send_pwm[i];
	   send_pwm[cnt_p++]=sum2;
		for(i=0;i<cnt_p;i++)
		USART1_Putc(send_pwm[i]);			
		}
}

void set_pid(u8 *buf)
{
	vs16 pid_temp;
	PID_ROL.P=24.5;   ///24~25
	PID_ROL.I=0.0;
	PID_ROL.D=6.8;    ///6.5~6.8
	PID_PIT.P=24.50;
	PID_PIT.I=0.0;
	PID_PIT.D=6.8;
	PID_YAW.P=0.0;
	PID_YAW.I=0.0;
	PID_YAW.D=-0.0;
	core_roll.P=0.03;
	core_roll.D=0.001;
	core_pitch.P=0.03;
	core_pitch.D=0.001;
	core_yaw.P=0.0;
	core_yaw.D=0.0;
	thr[0]=0;
	thr[1]=21;//19
	thr[2]=21;//19
	thr[3]=0;
	pid_temp = PID_ROL.P * 1000;
	buf[3]=BYTE1(pid_temp);//     3
	buf[4]=BYTE0(pid_temp);//        4
	pid_temp = PID_ROL.I * 1000;
	buf[5]=BYTE1(pid_temp);//     5
	buf[6]=BYTE0(pid_temp);//     6
	pid_temp = PID_ROL.D * 1000;
	buf[7]=BYTE1(pid_temp);//      7
	buf[8]=BYTE0(pid_temp);//      8
	pid_temp = PID_PIT.P * 1000;
	buf[9]=BYTE1(pid_temp);//      9
	buf[10]=BYTE0(pid_temp);//      10
	pid_temp = PID_PIT.I * 1000;
	buf[11]=BYTE1(pid_temp);//       11
	buf[12]=BYTE0(pid_temp);//       12
	pid_temp = PID_PIT.D * 1000;
	buf[13]=BYTE1(pid_temp);//       13
	buf[14]=BYTE0(pid_temp);//       14
	pid_temp = PID_YAW.P * 1000;
	buf[15]=BYTE1(pid_temp);//        15
	buf[16]=BYTE0(pid_temp);//        16
	pid_temp = PID_YAW.I * 1000;
	buf[17]=BYTE1(pid_temp);//         17
	buf[18]=BYTE0(pid_temp);//        18
	pid_temp = PID_YAW.D * 1000;
	buf[19]=BYTE1(pid_temp);//          19
	buf[20]=BYTE0(pid_temp);//         20
	buf[21]=thr[0];
	buf[22]=thr[1];
	buf[23]=thr[2];
	buf[24]=thr[3];
	pid_temp = core_roll.P * 1000;
	buf[25]=BYTE0(pid_temp);//     3
	pid_temp = core_roll.D * 1000;
	buf[26]=BYTE0(pid_temp);//      7
  pid_temp = core_pitch.P * 1000;
	buf[27]=BYTE0(pid_temp);//     3
	pid_temp = core_roll.D * 1000;
	buf[28]=BYTE0(pid_temp);//      7
	pid_temp = core_yaw.P * 1000;
	buf[29]=BYTE0(pid_temp);//     3
	pid_temp = core_yaw.D * 1000;
	buf[30]=BYTE0(pid_temp);//      7
}