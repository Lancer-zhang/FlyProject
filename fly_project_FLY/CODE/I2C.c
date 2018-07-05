#include "I2C.h"
#include "delay.h"

void Init_I2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C_SCL|I2C_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	I2C_SCL_H;
	I2C_SDA_H;
}
void Init_I2C2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C_SCL2|I2C_SDA2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	I2C_SCL2_H;
	I2C_SDA2_H;
}
void I2C_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
void I2C_SDA2_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
void I2C_SDA2_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=I2C_SDA2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
//产生起始信号
void I2C_Start(void)
{
    I2C_SDA_OUT();
	
	I2C_SDA_H;
	I2C_SCL_H;
	delay_us(5);
	I2C_SDA_L;
	delay_us(6);
	I2C_SCL_L;
}
void I2C_Start2(void)
{
    I2C_SDA2_OUT();
	
	I2C_SDA2_H;
	I2C_SCL2_H;
	delay_us(5);
	I2C_SDA2_L;
	delay_us(6);
	I2C_SCL2_L;
}
//产生停止信号
void I2C_Stop(void)
{
   I2C_SDA_OUT();

   I2C_SCL_L;
   I2C_SDA_L;
   I2C_SCL_H;
   delay_us(6);
   I2C_SDA_H;
   delay_us(6);
}
void I2C_Stop2(void)
{
   I2C_SDA2_OUT();

   I2C_SCL2_L;
   I2C_SDA2_L;
   I2C_SCL2_H;
   delay_us(6);
   I2C_SDA2_H;
   delay_us(6);
}
//主机产生应答信号ACK
void I2C_Ack(void)
{
   I2C_SCL_L;
   I2C_SDA_OUT();
   I2C_SDA_L;
   delay_us(2);
   I2C_SCL_H;
   delay_us(5);
   I2C_SCL_L;
}
void I2C_Ack2(void)
{
   I2C_SCL2_L;
   I2C_SDA2_OUT();
   I2C_SDA2_L;
   delay_us(2);
   I2C_SCL2_H;
   delay_us(5);
   I2C_SCL2_L;
}
//主机不产生应答信号NACK
void I2C_NAck(void)
{
   I2C_SCL_L;
   I2C_SDA_OUT();
   I2C_SDA_H;
   delay_us(2);
   I2C_SCL_H;
   delay_us(5);
   I2C_SCL_L;
}
void I2C_NAck2(void)
{
   I2C_SCL2_L;
   I2C_SDA2_OUT();
   I2C_SDA2_H;
   delay_us(2);
   I2C_SCL2_H;
   delay_us(5);
   I2C_SCL2_L;
}
//等待从机应答信号
//返回值：1 接收应答失败
//		  0 接收应答成功
u8 I2C_Wait_Ack(void)
{
	u8 tempTime=0;

	I2C_SDA_IN();

	I2C_SDA_H;
	delay_us(1);
	I2C_SCL_H;
	delay_us(1);

	while(GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA))
	{
		tempTime++;
		if(tempTime>250)
		{
			I2C_Stop();
			return 1;
		}	 
	}

	I2C_SCL_L;
	return 0;
}
u8 I2C_Wait_Ack2(void)
{
	u8 tempTime2=0;

	I2C_SDA2_IN();

	I2C_SDA2_H;
	delay_us(1);
	I2C_SCL2_H;
	delay_us(1);

	while(GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA2))
	{
		tempTime2++;
		if(tempTime2>250)
		{
			I2C_Stop2();
			return 1;
		}	 
	}

	I2C_SCL2_L;
	return 0;
}
u8 I2C_Receive_Byte()
{
    u8 i;
    u8 dat = 0;
    I2C_SDA_H;
	I2C_SCL_L; 
    delay_us(2);
	I2C_SDA_IN();
    for (i=0; i<8; i++)         
    {	
	    
		I2C_SCL_H;
	    delay_us(2);
        dat <<= 1;              
        delay_us(2); 			
        dat |= 0X01;                          
        I2C_SCL_L;               
        delay_us(2);          
    }
		I2C_SDA_OUT();
    return dat;
}
u8 I2C_Receive_Byte2()
{
    u8 i;
    u8 dat = 0;
    I2C_SDA2_H;
	I2C_SCL2_L; 
    delay_us(2);
	I2C_SDA2_IN();
    for (i=0; i<8; i++)         
    {	
	    
		I2C_SCL2_H;
	    delay_us(2);
        dat <<= 1;              
        delay_us(2); 			
        dat |= 0X01;                          
        I2C_SCL2_L;               
        delay_us(2);          
    }
		I2C_SDA2_OUT();
    return dat;
}
//I2C 发送一个字节
void I2C_Send_Byte(u8 txd)
{
	u8 i;
	I2C_SDA_OUT();
	I2C_SCL_L;//拉低时钟开始数据传输
	for(i=0;i<8;i++)
	{
		if((txd&0x80)>0) //0x80  1000 0000
			I2C_SDA_H;
		else
			I2C_SDA_L;

		txd<<=1;
		I2C_SCL_H;
		delay_us(2); //发送数据
		I2C_SCL_L;
		delay_us(2);
	}
}
void I2C_Send_Byte2(u8 txd)
{
	u8 i;
	I2C_SDA2_OUT();
	I2C_SCL2_L;//拉低时钟开始数据传输
	for(i=0;i<8;i++)
	{
		if((txd&0x80)>0) //0x80  1000 0000
			I2C_SDA2_H;
		else
			I2C_SDA2_L;

		txd<<=1;
		I2C_SCL2_H;
		delay_us(2); //发送数据
		I2C_SCL2_L;
		delay_us(2);
	}
}
//I2C 读取一个字节

u8 I2C_Read_Byte(u8 ack)
{
   u8 i=0,receive=0;

   I2C_SDA_IN();
   for(i=0;i<8;i++)
   {
   		I2C_SCL_L;
		delay_us(2);
		I2C_SCL_H;
		receive<<=1;
		if(GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA))
		   receive++;
		delay_us(1);	
   }

   	if(ack==0)
	   	I2C_NAck();
	else
		I2C_Ack();

	return receive;
}
u8 I2C_Read_Byte2(u8 ack)
{
   u8 i=0,receive=0;

   I2C_SDA2_IN();
   for(i=0;i<8;i++)
   {
   		I2C_SCL2_L;
		delay_us(2);
		I2C_SCL2_H;
		receive<<=1;
		if(GPIO_ReadInputDataBit(GPIO_I2C,I2C_SDA2))
		   receive++;
		delay_us(1);	
   }

   	if(ack==0)
	   	I2C_NAck2();
	else
		I2C_Ack2();

	return receive;
}


