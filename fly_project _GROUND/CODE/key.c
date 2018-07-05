/**
  ******************************************************************************
  * 文件名    key.c 
  * 功能      初始化KEY端口  
  * 库版本    V3.5.0
  * 日期      2012-11-13
  * 实验平台  小苗板
  * 硬件连接  KEY1-PC5
  			  KEY2-PC6
			  KEY3-PC7
			  KEY4-PC9 	
  ******************************************************************************
  * By：第九单片机论坛 nibutaiguai
  ****************************************************************************** 
  */ 
#include "key.h"

void KEY_GPIO_Config(void)
{	
	/*定义一个GPIO_InitTypeDef 类型的结构体，名字叫GPIO_InitStructure*/ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/*开启GPIO的外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	/*选择要用的GPIO引脚*/			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	
	/*设置引脚模式*/				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 

	/*调用库函数，初始化GPIO*/
	GPIO_Init(GPIOA, &GPIO_InitStructure);							
}

