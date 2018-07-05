/**
  ******************************************************************************
  * 文件名    ADC.c 
  * 功能      初始化ADC1
  * 库版本    V3.5.0
  * 日期      2012-11-15
  * 实验平台  小苗板
  * 硬件连接  电位器——PC4
  ******************************************************************************
  * By：第九单片机论坛 nibutaiguai
  ****************************************************************************** 
  */ 
#include "ADC.h"
extern __IO uint16_t ADC_ConvertedValue;
/**************************************************************
 ** 函数名 :ADC_Config
 ** 功能   :ADC初始化
 ** 输入   :无
 ** 输出   :无
 ** 返回   :无
 ** 注意   :       
***************************************************************/
void ADC_Config(void)
{
	ADC_InitTypeDef   ADC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE); //打开ADC1时钟，打开GPIOC时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_1;	 //PC4配置成模拟输入模式
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;		//独立模式
	ADC_InitStructure.ADC_ScanConvMode       = ENABLE;						//连续多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;						//连续转换
	ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;	//模数转换有软件启动
	ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;			//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel       = 1;							//扫描通道数，从1到16
	ADC_Init(ADC1, &ADC_InitStructure);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//9hz
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_71Cycles5);//通道X,采用时间为55.5周期,1代表规则通道第1个
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);
	ADC_DMACmd   (ADC1, ENABLE);   
	ADC_Cmd   (ADC1, ENABLE);              //使能ADC1  
  ADC_ResetCalibration(ADC1);	
	while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
	// //使能ADC1_14软件转换开始  
 DMA_DeInit(DMA1_Channel1);
 DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; 
 DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue; 
 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
 DMA_InitStructure.DMA_BufferSize = 1;
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
 DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
 DMA_InitStructure.DMA_Priority = DMA_Priority_High;
 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
 DMA_Init(DMA1_Channel1, &DMA_InitStructure);
 DMA_Cmd(DMA1_Channel1, ENABLE);

 ADC_SoftwareStartConvCmd(ADC1,ENABLE);

}
u16 Get_AD(void)   
{

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//检测转换结束标志位，等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
