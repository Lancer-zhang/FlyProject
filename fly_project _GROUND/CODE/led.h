#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#define LED1_ON   GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define LED1_OFF  GPIO_SetBits(GPIOC, GPIO_Pin_12)
#define LED2_ON   GPIO_ResetBits(GPIOD, GPIO_Pin_2)
#define LED2_OFF  GPIO_SetBits(GPIOD, GPIO_Pin_2)
#define LED3_ON   GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define LED3_OFF  GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define LED4_ON   GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define LED4_OFF  GPIO_SetBits(GPIOB, GPIO_Pin_3)
void LED_GPIO_Config(void);	

#endif
