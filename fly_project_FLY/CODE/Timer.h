#ifndef __Timer_H
#define	__Timer_H

#include "stm32f10x.h"
extern u16 cnt_2ms;
extern u16 cnt_5ms;
extern u16 cnt_10ms;
extern u16 cnt_20ms;
void TIM3_Config(void);
//void TIM2_delay(u16 s);
#endif
