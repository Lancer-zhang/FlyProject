/**
  ******************************************************************************
  * �ļ���    USART.c 
  * ����      ��ʼ��USART1 
  * ��汾    V3.5.0
  * ����      2012-11-13
  * ʵ��ƽ̨  С���
  * Ӳ������  TX-PA9
  			  RX-PA10	
  ******************************************************************************
  * By���ھŵ�Ƭ����̳ nibutaiguai
  ****************************************************************************** 
  */ 
#include "USART.h"
#include "delay.h"
void USART1_Config(void)
{	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
		
	/*ʹ��GPIOAʱ�ӣ�ʹ�ܴ���1ʱ��*/
	SystemInit();//72m
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	delay_ms(100);
	/*A9 TX�����������*/ 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*A10 RX�������� */ 
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
	/*����USART1�Ĺ���ģʽ*/
	USART_InitStructure.USART_BaudRate = 115200;                 //������9600         
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�8bit										
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	   //ֹͣλ1λ									
	USART_InitStructure.USART_Parity = USART_Parity_No;		   //����żУ��							
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ʹ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ��������							
    USART_Init(USART1, &USART_InitStructure);   //��ʼ������ USART1   
    USART_Cmd(USART1, ENABLE);					//ʹ��USART����
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//ʹ�ܽ����ж�
	USART_ClearFlag(USART1,USART_FLAG_TC);
}
/**************************************************************
 ** ������ :fputc
 ** ����   :�ض���c�⺯��printf��USART
 ** ����   :��
 ** ���   :��
 ** ����   :��
 ** ע��   :��printf����
***************************************************************/
int fputc(int ch, FILE *f)
{
/* ��Printf���ݷ������� */
  USART_SendData(USART1, (unsigned char) ch);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
  return (ch);
}
/**************************************************************
 ** ������ :USART1_Putc
 ** ����   :��USART1_Putc�������ݴ�ӡ������
 ** ����   :��
 ** ���   :��
 ** ����   :��
 ** ע��   :��
***************************************************************/
void USART1_Putc(unsigned char c)
{
    USART_SendData(USART1, c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
}
/**************************************************************
 ** ������ :USART1_Puts
 ** ����   :��USART1_Puts�������ݴ�ӡ������
 ** ����   :��
 ** ���   :��
 ** ����   :��
 ** ע��   :��
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
