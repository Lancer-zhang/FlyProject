#include "pwm.h"
#include "stm32f10x_tim.h"
void TIM2_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*GPIOB Configuration: TIM3 channel 3 and 4 as alternate function push-pull */
  
}
void TIM2_Mode_Config(u16 period_value,u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseStructure.TIM_Period = period_value;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //288����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ36MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��1

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ʹ��ͨ��2

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��3

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4

  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // ʹ��TIM3���ؼĴ���ARR

  /* TIM3 enable counter */
  TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��3	
}

void TIM2_PWM_Init(void)
{
	TIM2_GPIO_Config();
	TIM2_Mode_Config(25000,0,0,0,0);	
}
