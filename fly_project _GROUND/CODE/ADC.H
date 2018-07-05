#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"
#define ADC1_DR_Address ((uint32_t)0x4001244C)

//__IO uint16_t ADC_ConvertedValue;
void ADC_Config(void);
u16 Get_AD(void);
#endif

