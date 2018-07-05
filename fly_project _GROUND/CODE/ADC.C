/**
  ******************************************************************************
  * �ļ���    ADC.c 
  * ����      ��ʼ��ADC1
  * ��汾    V3.5.0
  * ����      2012-11-15
  * ʵ��ƽ̨  С���
  * Ӳ������  ��λ������PC4
  ******************************************************************************
  * By���ھŵ�Ƭ����̳ nibutaiguai
  ****************************************************************************** 
  */ 
#include "ADC.h"
extern __IO uint16_t ADC_ConvertedValue;
/**************************************************************
 ** ������ :ADC_Config
 ** ����   :ADC��ʼ��
 ** ����   :��
 ** ���   :��
 ** ����   :��
 ** ע��   :       
***************************************************************/
void ADC_Config(void)
{
	ADC_InitTypeDef   ADC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE); //��ADC1ʱ�ӣ���GPIOCʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_1;	 //PC4���ó�ģ������ģʽ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;		//����ģʽ
	ADC_InitStructure.ADC_ScanConvMode       = ENABLE;						//������ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;						//����ת��
	ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;	//ģ��ת������������
	ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;			//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel       = 1;							//ɨ��ͨ��������1��16
	ADC_Init(ADC1, &ADC_InitStructure);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//9hz
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_71Cycles5);//ͨ��X,����ʱ��Ϊ55.5����,1��������ͨ����1��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);
	ADC_DMACmd   (ADC1, ENABLE);   
	ADC_Cmd   (ADC1, ENABLE);              //ʹ��ADC1  
  ADC_ResetCalibration(ADC1);	
	while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
	// //ʹ��ADC1_14����ת����ʼ  
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

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//���ת��������־λ���ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}