/**
  ******************************************************************************
  * �ļ���    key.c 
  * ����      ��ʼ��KEY�˿�  
  * ��汾    V3.5.0
  * ����      2012-11-13
  * ʵ��ƽ̨  С���
  * Ӳ������  KEY1-PC5
  			  KEY2-PC6
			  KEY3-PC7
			  KEY4-PC9 	
  ******************************************************************************
  * By���ھŵ�Ƭ����̳ nibutaiguai
  ****************************************************************************** 
  */ 
#include "key.h"

void KEY_GPIO_Config(void)
{	
	/*����һ��GPIO_InitTypeDef ���͵Ľṹ�壬���ֽ�GPIO_InitStructure*/ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/*����GPIO������ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	/*ѡ��Ҫ�õ�GPIO����*/			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	
	/*��������ģʽ*/				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 

	/*���ÿ⺯������ʼ��GPIO*/
	GPIO_Init(GPIOA, &GPIO_InitStructure);							
}

