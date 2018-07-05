#ifndef _I2C_H
#define _I2C_H
#include "delay.h"

//如果移植程序时只要改一下三个地方就行了
#define I2C_SCL GPIO_Pin_6
#define I2C_SDA GPIO_Pin_7
#define I2C_SCL2 GPIO_Pin_10
#define I2C_SDA2 GPIO_Pin_11
#define GPIO_I2C GPIOB

#define I2C_SCL_H GPIO_SetBits(GPIO_I2C,I2C_SCL)
#define I2C_SCL_L GPIO_ResetBits(GPIO_I2C,I2C_SCL)

#define I2C_SDA_H GPIO_SetBits(GPIO_I2C,I2C_SDA)
#define I2C_SDA_L GPIO_ResetBits(GPIO_I2C,I2C_SDA)

#define I2C_SCL2_H GPIO_SetBits(GPIO_I2C,I2C_SCL2)
#define I2C_SCL2_L GPIO_ResetBits(GPIO_I2C,I2C_SCL2)

#define I2C_SDA2_H GPIO_SetBits(GPIO_I2C,I2C_SDA2)
#define I2C_SDA2_L GPIO_ResetBits(GPIO_I2C,I2C_SDA2)

void Init_I2C(void);
void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
u8   I2C_Wait_Ack(void);
void I2C_Send_Byte(u8 txd);
u8   I2C_Read_Byte(u8 ack);
u8 I2C_Receive_Byte(void);

void Init_I2C2(void);
void I2C_SDA2_OUT(void);
void I2C_SDA2_IN(void);
void I2C_Start2(void);
void I2C_Stop2(void);
void I2C_Ack2(void);
void I2C_NAck2(void);
u8   I2C_Wait_Ack2(void);
void I2C_Send_Byte2(u8 txd);
u8   I2C_Read_Byte2(u8 ack);
u8 I2C_Receive_Byte2(void);

#endif
