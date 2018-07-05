/* 
���ܣ�
�ṩHMC5883L ��ʼ�� ��ȡ�����Ƶ�ǰADCת�����
------------------------------------
 */

#include "HMC5883L.h"
#include "I2C.h"
#include "delay.h"
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;

int16_t  HMC5883_FIFO[3][11]; //�������˲�



void HMC5883_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_Start2();
	I2C_Send_Byte2(HMC58X3_ADDR);
	I2C_Wait_Ack2();
	I2C_Send_Byte2(reg_add);
	I2C_Wait_Ack2();
	I2C_Send_Byte2(reg_dat);
	I2C_Wait_Ack2();
	I2C_Stop2();
}
void HMC5883_ReadData(u8 reg_add,unsigned char*Read,u8 num)
{
	unsigned char i;
	
	I2C_Start2();
	I2C_Send_Byte2(HMC58X3_ADDR);
	I2C_Wait_Ack2();
	I2C_Send_Byte2(reg_add);
	I2C_Wait_Ack2();
	
	I2C_Start2();
	I2C_Send_Byte2(HMC58X3_ADDR+1);
	I2C_Wait_Ack2();
	
	for(i=0;i<(num-1);i++){
		*Read=I2C_Read_Byte2(1);
		Read++;
	}
	*Read=I2C_Read_Byte2(0);
	I2C_Stop2();
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   unsigned char HMC5883_IS_newdata(void)
*��������:	   ��ȡDRDY ���ţ��ж��Ƿ������һ��ת��
 Low for 250 ��sec when data is placed in the data output registers. 
���������  ��
���������  ������ת���������1  ������� 0
******************************************************************************
unsigned char HMC5883_IS_newdata(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC58X3_FIFO_init(void)
*��������:	   ������ȡ100�����ݣ��Գ�ʼ��FIFO����
���������  ��
���������  ��
******************************************************************************
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //��ʱ�ٶ�ȡ����

  }
}

**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*��������:	   ����һ�����ݵ�FIFO����
���������  �������������Ӧ��ADCֵ
���������  ��
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=x;
	HMC5883_FIFO[1][9]=y;
	HMC5883_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//��ƽ��ֵ����

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5883L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) {
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   HMC5883_ReadData(HMC58X3_R_XM,vbuff,6);
   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5883_FIFO[0][10];
   *y = HMC5883_FIFO[1][10];
   *z = HMC5883_FIFO[2][10];
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC58X3_mgetValues(float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void HMC58X3_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);
  arry[0]= HMC5883_lastx=(float)(xr);
  arry[1]= HMC5883_lasty=(float)(yr);
  arry[2]= HMC5883_lastz=(float)(zr);
}


/*void Multiple_Read_HMC5883L(u8 *BUF)
{   
	  u8 i;
    I2C_Start();                          //��ʼ�ź�
    I2C_Send_Byte(HMC58X3_ADDR);           //�����豸��ַ+д�ź�
    I2C_Ack();
	I2C_Send_Byte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
    I2C_Start();                          //��ʼ�ź�
    I2C_Send_Byte(HMC58X3_ADDR+1);         //�����豸��ַ+���ź�
	  for (i=0; i<6; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
    {
        *BUF = I2C_Receive_Byte();          //BUF[0]�洢����
		BUF++;
        if (i == 5)
        {
           I2C_Ack();                //���һ��������Ҫ��NOACK
        }
        else
        {
          I2C_NAck();                 //��ӦACK
        }
    }
    I2C_Stop();                           //ֹͣ�ź�
    delay_ms(5);
}  */
//��ʼ��HMC5883��������Ҫ��ο�pdf�����޸�****  
void Init_HMC5883L(void)
{
   HMC5883_WriteReg(HMC58X3_R_MODE,0x00);
   HMC5883_WriteReg(HMC58X3_R_CONFB,0xE0);  
}

 // HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
//  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
 // HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

//------------------End of File----------------------------
