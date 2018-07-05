/**
  ******************************************************************************
  * �ļ���    led.c 
  * ����      ���������ĸ�LED  
  * ��汾    V3.5.0
  * ����      2012-11-13
  * ʵ��ƽ̨  С���
  * Ӳ������  LED1-PC0
  			  LED2-PC1
			  LED3-PC2
			  LED4-PC3 	
  ******************************************************************************
  * By���ھŵ�Ƭ����̳ nibutaiguai
  ****************************************************************************** 
  */ 
#include "led.h"

void LED_GPIO_Config(void)
{	
	/*����һ��GPIO_InitTypeDef ���͵Ľṹ�壬���ֽ�GPIO_InitStructure*/ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/*����GPIO������ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);
	
	/*ѡ��Ҫ�õ�GPIO����*/			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	/*��������ģʽ*/				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	/*���������ٶ�*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*���ÿ⺯������ʼ��GPIO*/
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*GPIO������ߵ͵�ƽ*/
	GPIO_SetBits(GPIOD,GPIO_Pin_2);	
	GPIO_SetBits(GPIOB,GPIO_Pin_4);	
	GPIO_SetBits(GPIOB,GPIO_Pin_3);	
	GPIO_SetBits(GPIOC,GPIO_Pin_12);						  						 
	
}
