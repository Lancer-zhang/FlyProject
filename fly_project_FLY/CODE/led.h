#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#define ON  0
#define OFF 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	                           \
					GPIO_SetBits(GPIOD,GPIO_Pin_2);\
				else		                       \
					GPIO_ResetBits(GPIOD,GPIO_Pin_2)
void LED_GPIO_Config(void);	

#endif
