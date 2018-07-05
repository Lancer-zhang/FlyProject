#include "IMU.h"
#include "delay.h"
#include "timer.h"
volatile uint32_t lastUpdate, now;
volatile float exInt, eyInt, ezInt;  // ������
//volatile float exdev, eydev, ezdev;
volatile float q0, q1, q2, q3; // ȫ����Ԫ��
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
//extern float gyro[3];
 //#define q30  1073741824.0f
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_init(void)
*��������:	  ��ʼ��IMU���	
			  ��ʼ������������
			  ��ʼ����Ԫ��
			  ����������
			  ����ϵͳʱ��
�����������
���������û��
*******************************************************************************/

void IMU_init(void)
{	 
	MPU6050_Init();
	Init_HMC5883L();

	delay_ms(50);
	MPU6050_Init();
	Init_HMC5883L();
	delay_ms(50);

//	Initial_Timer3();
	// initialize quaternion
  	q0 = 1.0f;  //��ʼ����Ԫ��
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
	
 // 	lastUpdate = micros();//����ʱ��
 // 	now = micros();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getValues(float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
��������� �������ŵ������׵�ַ
���������û��
*******************************************************************************/
#define new_weight 0.35f
#define old_weight 0.65f
#define b0 0.1883633f
#define a1 1.023694f
#define a2 0.2120577f
void IMU_getValues(float * values) {  
	int16_t accgyroval[6];
	static  float lastacc[3]= {0,0,0};
	int i;
	//��ȡ���ٶȺ������ǵĵ�ǰADC
    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
    for(i = 0; i<6; i++) {
      if(i < 3) {
		//		LPF_2nd((float) accgyroval[i]);
        values[i] = (float) accgyroval[i]* new_weight +lastacc[i] * old_weight ;
                lastacc[i] = values[i];
      }
      else {
        values[i] = ((float) accgyroval[i]) / 16.4f; //ת�ɶ�ÿ��
		//�����Ѿ������̸ĳ��� 1000��ÿ��  32.8 ��Ӧ 1��ÿ��
      }
    }
		
//	Multiple_Read_HMC5883L();
    HMC58X3_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
}

float LPF_2nd(float newdata)
{
	 float lpf_2nd_data;
	float preout=0,lastout=0;
	lpf_2nd_data = newdata * b0 + lastout * a1 - preout * a2;
	preout = lastout;
	lastout = lpf_2nd_data;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
#define Kp 40.0f   // ���ٶȼ�(������)���������ʱ�������50 
#define Ki 0.75f   //�������������ʵĻ������� 0.75

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{

  float norm;
  volatile float halfT;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
	float delta_2=0;
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
/* now = GET_NOWTIME();  //��ȡʱ��
  if(now < lastUpdate)
  { //��ʱ��������ˡ�
                halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);        
                lastUpdate = now;
                //return ;
  }
  else
    {
                halfT =  ((float)(now - lastUpdate) / 2000000.0f);
          }
  lastUpdate = now;        //����ʱ��

 */ 
	halfT= 0.001f   ;
norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux ͨ���ļ���ο�����
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);				  	// ���Ʒ��������
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
 

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * halfT;
  eyInt = eyInt + ey * halfT;	
  ezInt = ezInt + ez * halfT;

  //exdev=(ex[1]-ex[0]) / halfT;
  //eydev=(ey[1]-ey[0]) / halfT;
 // ezdev=(ez[1]-ez[0]) / halfT;
 // adjusted gyroscope measurements
  gx = gx+ Kp*ex + Ki*exInt; //+ Kd*exdev;
  gy = gy+ Kp*ey + Ki*eyInt; //+ Kd*eydev;
  gz = gz+ Kp*ez + Ki*ezInt; //+ Kd*ezdev;

  }
delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
  // integrate quaternion rate and normalise
//  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨�����ױϿ���
  q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;	
  // normalise quaternion
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getQ(float * q)
*��������:	 ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
��������� ��Ҫ�����Ԫ���������׵�ַ
���������û��			
*******************************************************************************/
extern float mygetqval[9];	//���ڴ�Ŵ�����ת�����������
void IMU_getQ(float * q) {

  IMU_getValues(mygetqval);	 
  //�������ǵĲ���ֵת�ɻ���ÿ��
  //���ٶȺʹ����Ʊ��� ADCֵ������Ҫת��
 IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0;///q30; //���ص�ǰֵ
  q[1] = q1;///q30;
  q[2] = q2;///q30;
  q[3] = q3;///q30;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/

void IMU_getYawPitchRoll(float * angles) {
  float q[4]; //����Ԫ��
	float angleAx=0,angleAy=0;
  volatile float gx=0.0, gy=0.0, gz=0.0; //������������
  IMU_getQ(q); //����ȫ����Ԫ��
  angleAx=atan(mygetqval[0]/sqrt(mygetqval[1]*mygetqval[1]+mygetqval[2]*mygetqval[2]))* 180/M_PI;
	angleAx=atan(mygetqval[1]/sqrt(mygetqval[0]*mygetqval[0]+mygetqval[2]*mygetqval[2]))* 180/M_PI;
  angles[2] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
	angles[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	
	
	//�����HMC5883L �����������
 // angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw+=mygetqval[5]/200;
    angles[0] +=mygetqval[5]/200;//�����HMC5883L  ע�͵���� ���������
	angles[2] =0.98*(angles[2]-mygetqval[4]*0.002)+0.002*angleAy;
	angles[1] =0.98*(angles[1]-mygetqval[3]*0.002)+0.002*angleAx;
	
	if(angles[1]>90||angles[1]<-90)
  {
  if(angles[2]>0)
	angles[2]=180-angles[2];
	if(angles[2]<0)
	angles[2]=-(180+angles[2]);
  }
	
	
	
	if(angles[0] > 180)
		angles[0] -=360;
	else if(angles[0] <-180)
		angles[0] +=360;
} 

//------------------End of File----------------------------
