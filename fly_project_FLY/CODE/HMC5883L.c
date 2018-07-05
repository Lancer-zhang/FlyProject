/* 
功能：
提供HMC5883L 初始化 读取磁力计当前ADC转换结果
------------------------------------
 */

#include "HMC5883L.h"
#include "I2C.h"
#include "delay.h"
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;

int16_t  HMC5883_FIFO[3][11]; //磁力计滤波



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
/**************************实现函数********************************************
*函数原型:	   unsigned char HMC5883_IS_newdata(void)
*功　　能:	   读取DRDY 引脚，判断是否完成了一次转换
 Low for 250 μsec when data is placed in the data output registers. 
输入参数：  无
输出参数：  如果完成转换，则输出1  否则输出 0
******************************************************************************
unsigned char HMC5883_IS_newdata(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

**************************实现函数********************************************
*函数原型:	   void HMC58X3_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
******************************************************************************
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //延时再读取数据

  }
}

**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
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
	for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//将平均值更新

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


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
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


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_mgetValues(float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
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
    I2C_Start();                          //起始信号
    I2C_Send_Byte(HMC58X3_ADDR);           //发送设备地址+写信号
    I2C_Ack();
	I2C_Send_Byte(0x03);                   //发送存储单元地址，从0x3开始	
    I2C_Start();                          //起始信号
    I2C_Send_Byte(HMC58X3_ADDR+1);         //发送设备地址+读信号
	  for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
    {
        *BUF = I2C_Receive_Byte();          //BUF[0]存储数据
		BUF++;
        if (i == 5)
        {
           I2C_Ack();                //最后一个数据需要回NOACK
        }
        else
        {
          I2C_NAck();                 //回应ACK
        }
    }
    I2C_Stop();                           //停止信号
    delay_ms(5);
}  */
//初始化HMC5883，根据需要请参考pdf进行修改****  
void Init_HMC5883L(void)
{
   HMC5883_WriteReg(HMC58X3_R_MODE,0x00);
   HMC5883_WriteReg(HMC58X3_R_CONFB,0xE0);  
}

 // HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
//  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
 // HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

//------------------End of File----------------------------
