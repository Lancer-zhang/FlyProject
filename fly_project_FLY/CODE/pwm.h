#ifndef __PWM_H
#define	__PWM_H

#include "stm32f10x.h"
void TIM2_GPIO_Config(void);
void TIM2_Mode_Config(u16 period_value,u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val);
void TIM2_PWM_Init(void);
//void TIM3_PWM_OUTPUT(u16 DR1,u16 DR2,u16 DR3,u16 DR4);

#endif /* __PWM_OUTPUT_H */
