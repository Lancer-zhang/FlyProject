/**
  ******************************************************************************
  * �ļ���    Timer.c 
  * ����      ��ʼ��ͨ�ö�ʱ��TIM2��ʵ��TIM2��ʱ���� 
  * ��汾    V3.5.0
  * ����      2012-11-13
  * ʵ��ƽ̨  С���
  * Ӳ������  
  ******************************************************************************
  * By���ھŵ�Ƭ����̳ nibutaiguai
  ****************************************************************************** 
  */ 
#include "Timer.h"
//extern u16 Time3=0;

void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef         NVIC_InitStructure;
	/*ʹ��TIM2ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	           //ѡ����1
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	/*�Զ���װ��ֵ1000--��ʱʱ��(1*7200/72M)s*/
	TIM_TimeBaseStructure.TIM_Period = 10; //1ms
	/*Ԥ��Ƶֵ��+1Ϊ��Ƶϵ��*/
	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1);  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	/*TIM���ϼ���ģʽ*/
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/*����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ*/  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 	/*ʹ��TIM2�ж�Դ*/
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
	/*ʹ��TIMx����*/
	TIM_Cmd(TIM3, ENABLE);  
							 	
}
void TIM3_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) //�Ƿ����ж�
	{	
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);    //����жϴ�����λ
  		cnt_2ms++;
		cnt_5ms++;
		cnt_10ms++;
		cnt_20ms++;
	}		 	
}

