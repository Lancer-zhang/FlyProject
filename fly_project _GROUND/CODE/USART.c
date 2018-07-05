/**
  ******************************************************************************
  * 文件名    USART.c 
  * 功能      初始化USART1 
  * 库版本    V3.5.0
  * 日期      2012-11-13
  * 实验平台  小苗板
  * 硬件连接  TX-PA9
  			  RX-PA10	
  ******************************************************************************
  * By：第九单片机论坛 nibutaiguai
  ****************************************************************************** 
  */ 
#include "USART.h"
#include "delay.h"
void USART1_Config(void)
{	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
		
	/*使能GPIOA时钟，使能串口1时钟*/
	SystemInit();//72m
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	delay_ms(100);
	/*A9 TX复用推挽输出*/ 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*A10 RX浮空输入 */ 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	delay_ms(100);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*USART1*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	delay_ms(100);
	/*配置USART1的工作模式*/
	USART_InitStructure.USART_BaudRate = 115200;                 //波特率9600         
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长8bit										
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	   //停止位1位									
	USART_InitStructure.USART_Parity = USART_Parity_No;		   //无奇偶校验							
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发使能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件流控制							
    USART_Init(USART1, &USART_InitStructure);   //初始化外设 USART1   
    USART_Cmd(USART1, ENABLE);					//使能USART外设
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//使能接收中断
	USART_ClearFlag(USART1,USART_FLAG_TC);
}
/**************************************************************
 ** 函数名 :fputc
 ** 功能   :重定向c库函数printf到USART
 ** 输入   :无
 ** 输出   :无
 ** 返回   :无
 ** 注意   :由printf调用
***************************************************************/
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
  return (ch);
}
/**************************************************************
 ** 函数名 :USART1_Putc
 ** 功能   :将USART1_Putc（）内容打印到串口
 ** 输入   :无
 ** 输出   :无
 ** 返回   :无
 ** 注意   :无
***************************************************************/
void USART1_Putc(unsigned char c)
{
    USART_SendData(USART1, c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
}
/**************************************************************
 ** 函数名 :USART1_Puts
 ** 功能   :将USART1_Puts（）内容打印到串口
 ** 输入   :无
 ** 输出   :无
 ** 返回   :无
 ** 注意   :无
***************************************************************/
void USART1_Puts(char * str)
{
    while(*str)
    {
        USART_SendData(USART1, *str++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

void USART1_IRQHandler(void)
{
	 u16 str1;
     
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
     USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
     str1=USART_ReceiveData(USART1);
     USART_SendData(USART1, str1);
     while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
     USART_ITConfig( USART1,USART_IT_RXNE, ENABLE);
  }
}
