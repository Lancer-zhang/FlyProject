#include "mpu6050.h"
#include "delay.h"
void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_Start();
	I2C_Send_Byte(MPU6050_SLAVE_ADDRESS);
	I2C_Wait_Ack();
	I2C_Send_Byte(reg_add);
	I2C_Wait_Ack();
	I2C_Send_Byte(reg_dat);
	I2C_Wait_Ack();
	I2C_Stop();
}

void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num)
{
	unsigned char i;
	
	I2C_Start();
	I2C_Send_Byte(MPU6050_SLAVE_ADDRESS);
	I2C_Wait_Ack();
	I2C_Send_Byte(reg_add);
	I2C_Wait_Ack();
	
	I2C_Start();
	I2C_Send_Byte(MPU6050_SLAVE_ADDRESS+1);
	I2C_Wait_Ack();
	
	for(i=0;i<(num-1);i++){
		*Read=I2C_Read_Byte(1);
		Read++;
	}
	*Read=I2C_Read_Byte(0);
	I2C_Stop();
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //½â³ýÐÝÃß×´Ì¬    
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //ÍÓÂÝÒÇ²ÉÑùÂÊ  0x00
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //ÅäÖÃ¼ÓËÙ¶È´«¸ÐÆ÷¹¤×÷ÔÚ16GÄ£Ê½
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //ÍÓÂÝÒÇ×Ô¼ì¼°²âÁ¿·¶Î§£¬µäÐÍÖµ£º0x18(²»×Ô¼ì£¬2000deg/s)
	MPU6050_WriteReg(MPU6050_RA_INT_PIN_CFG,0x02);
	MPU6050_WriteReg(MPU6050_RA_USER_CTRL,0x00);
//	MPU6050_WriteReg(MPU6050_RA_I2C_MST_DELAY_CTRL,0x00);//?£??
	}
/*void MPU6050ReadID(void)
{
	unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //¶ÁÆ÷¼þµØÖ·
     printf("%d\r\n",Re);
}	*/
void MPU6050ReadAcc(int16_t *accData)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
void MPU6050ReadGyro(int16_t *gyroData)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //¶ÁÈ¡ÎÂ¶ÈÖµ
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	u8 buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //¶ÁÈ¡ÎÂ¶ÈÖµ
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}
 uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
int16_t Ax_offset=0,Ay_offset=0,Az_offset=0;

/**************************ÊµÏÖº¯Êý********************************************
*º¯ÊýÔ­ÐÍ:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*¹¦¡¡¡¡ÄÜ:	    ½«ÐÂµÄADCÊý¾Ý¸üÐÂµ½ FIFOÊý×é£¬½øÐÐÂË²¨´¦Àí
*******************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){	//FIFO ²Ù×÷
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;//½«ÐÂµÄÊý¾Ý·ÅÖÃµ½ Êý¾ÝµÄ×îºóÃæ
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;

sum=0;
for(i=0;i<10;i++){	//Çóµ±Ç°Êý×éµÄºÏ£¬ÔÙÈ¡Æ½¾ùÖµ
   sum+=MPU6050_FIFO[0][i];
}
MPU6050_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[1][i];
}
MPU6050_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[2][i];
}
MPU6050_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[3][i];
}
MPU6050_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[4][i];
}
MPU6050_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[5][10]=sum/10;
}

int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  	int16_t ACCdata[3];
	int16_t GYROdata[3];
	if(1){
	MPU6050ReadAcc(ACCdata);
    MPU6050_Lastax=ACCdata[0];
    MPU6050_Lastay=ACCdata[1];
    MPU6050_Lastaz=ACCdata[2];
	MPU6050ReadGyro(GYROdata);
    MPU6050_Lastgx=GYROdata[0];
    MPU6050_Lastgy=GYROdata[1];
    MPU6050_Lastgz=GYROdata[2];
//	MPU6050_newValues(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);
	*ax  =ACCdata[0]-Ax_offset;//
	*ay  =ACCdata[1]-Ay_offset;//
	*az  =ACCdata[2]-Az_offset;//
	*gx  =GYROdata[0]-Gx_offset;//MPU6050_Lastgx;//
	*gy  =GYROdata[1]-Gy_offset;//MPU6050_Lastgy;//
	*gz  =GYROdata[2]-Gz_offset;// MPU6050_Lastgz;//
//			*ax  =MPU6050_FIFO[0][10]-Ax_offset;//
//	*ay  =MPU6050_FIFO[1][10]-Ay_offset;//
//	*az  =MPU6050_FIFO[2][10]-Az_offset;//
//	*gx  =MPU6050_FIFO[3][10]-Gx_offset;//MPU6050_Lastgx;//
//	*gy  =MPU6050_FIFO[4][10]-Gy_offset;//MPU6050_Lastgy;//
//	*gz  =MPU6050_FIFO[5][10]-Gz_offset;
	}
}
 /*
void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =MPU6050_FIFO[0][10];
	*ay  =MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];
	*gx  =MPU6050_FIFO[3][10]-Gx_offset;
	*gy = MPU6050_FIFO[4][10]-Gy_offset;
	*gz = MPU6050_FIFO[5][10]-Gz_offset;
}
 */
/**************************ÊµÏÖº¯Êý********************************************
*º¯ÊýÔ­ÐÍ:		void MPU6050_InitGyro_Offset(void)
*¹¦¡¡¡¡ÄÜ:	    ¶ÁÈ¡ MPU6050µÄÍÓÂÝÒÇÆ«ÖÃ
´ËÊ±Ä£¿éÓ¦¸Ã±»¾²Ö¹·ÅÖÃ¡£ÒÔ²âÊÔ¾²Ö¹Ê±µÄÍÓÂÝÒÇÊä³ö
*******************************************************************************/
void MPU6050_InitGyro_Offset(void)
{
	unsigned char i;
	int16_t temp[6];
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	int32_t	tempax=0,tempay=0,tempaz=0;
	Ax_offset=0;
	Ay_offset=0;
	Az_offset=0;
	Gx_offset=0;
	Gy_offset=0;
	Gz_offset=0;
	for(i=0;i<50;i++){
  		delay_us(100);
  		MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);

	}
 	for(i=0;i<100;i++){
		delay_us(800);
		MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];

	}

	Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
	Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
	Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];
	tempax/=100;
	tempay/=100;
	tempaz/=100;
	Ax_offset= tempax;
	Ay_offset= tempay;
	Az_offset= tempaz;
}

//------------------End of File----------------------------
