#include "IMU.h"
#include "delay.h"
#include "timer.h"
volatile uint32_t lastUpdate, now;
volatile float exInt, eyInt, ezInt;  // 误差积分
//volatile float exdev, eydev, ezdev;
volatile float q0, q1, q2, q3; // 全局四元数
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
//extern float gyro[3];
 //#define q30  1073741824.0f
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
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


/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关	
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
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
  	q0 = 1.0f;  //初始化四元数
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
	
 // 	lastUpdate = micros();//更新时间
 // 	now = micros();
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
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
	//读取加速度和陀螺仪的当前ADC
    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
    for(i = 0; i<6; i++) {
      if(i < 3) {
		//		LPF_2nd((float) accgyroval[i]);
        values[i] = (float) accgyroval[i]* new_weight +lastacc[i] * old_weight ;
                lastacc[i] = values[i];
      }
      else {
        values[i] = ((float) accgyroval[i]) / 16.4f; //转成度每秒
		//这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒
      }
    }
		
//	Multiple_Read_HMC5883L();
    HMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
}

float LPF_2nd(float newdata)
{
	 float lpf_2nd_data;
	float preout=0,lastout=0;
	lpf_2nd_data = newdata * b0 + lastout * a1 - preout * a2;
	preout = lastout;
	lastout = lpf_2nd_data;
}
/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 40.0f   // 加速度计(磁力计)的收敛速率比例增益50 
#define Ki 0.75f   //陀螺仪收敛速率的积分增益 0.75

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
/* now = GET_NOWTIME();  //读取时间
  if(now < lastUpdate)
  { //定时器溢出过了。
                halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);        
                lastUpdate = now;
                //return ;
  }
  else
    {
                halfT =  ((float)(now - lastUpdate) / 2000000.0f);
          }
  lastUpdate = now;        //更新时间

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

  // compute reference direction of flux 通量的计算参考方向
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);				  	// 估计方向的重力
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
  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// 整合四元数率	 四元数微分方程	四元数更新算法，二阶毕卡法
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

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有			
*******************************************************************************/
extern float mygetqval[9];	//用于存放传感器转换结果的数组
void IMU_getQ(float * q) {

  IMU_getValues(mygetqval);	 
  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
 IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0;///q30; //返回当前值
  q[1] = q1;///q30;
  q[2] = q2;///q30;
  q[3] = q3;///q30;
}


/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/

void IMU_getYawPitchRoll(float * angles) {
  float q[4]; //　四元数
	float angleAx=0,angleAy=0;
  volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
  IMU_getQ(q); //更新全局四元数
  angleAx=atan(mygetqval[0]/sqrt(mygetqval[1]*mygetqval[1]+mygetqval[2]*mygetqval[2]))* 180/M_PI;
	angleAx=atan(mygetqval[1]/sqrt(mygetqval[0]*mygetqval[0]+mygetqval[2]*mygetqval[2]))* 180/M_PI;
  angles[2] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
	angles[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
	
	
	//如果有HMC5883L 保留下面语句
 // angles[0] = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw+=mygetqval[5]/200;
    angles[0] +=mygetqval[5]/200;//如果有HMC5883L  注释掉这个 保留上面的
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
