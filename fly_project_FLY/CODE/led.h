#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#define ON  0
#define OFF 1

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	if (a)	                           \
					GPIO_SetBits(GPIOD,GPIO_Pin_2);\
				else		                       \
					GPIO_ResetBits(GPIOD,GPIO_Pin_2)
void LED_GPIO_Config(void);	

#endif
