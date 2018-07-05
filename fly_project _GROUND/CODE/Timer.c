/**
  ******************************************************************************
  * 文件名    Timer.c 
  * 功能      初始化通用定时器TIM2，实现TIM2定时功能 
  * 库版本    V3.5.0
  * 日期      2012-11-13
  * 实验平台  小苗板
  * 硬件连接  
  ******************************************************************************
  * By：第九单片机论坛 nibutaiguai
  ****************************************************************************** 
  */ 
#include "Timer.h"
u16 Time2=0;

void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef         NVIC_InitStructure;
	/*使能TIM2时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	           //选择组1
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	/*自动重装载值1000--定时时间(1*7200/72M)s*/
	TIM_TimeBaseStructure.TIM_Period = 100; //10ms
	/*预分频值，+1为分频系数*/
	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1);  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	/*TIM向上计数模式*/
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/*根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位*/  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 	/*使能TIM2中断源*/
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
	/*使能TIMx外设*/
	TIM_Cmd(TIM2, ENABLE);  
							 	
}
void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) //是否发生中断
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);    //清除中断待处理位
  		Time2++;
	}		 	
}

