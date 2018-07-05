#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f10x.h"
#include "I2C.h"
#include "delay.h"
//#include "LED.h"

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0x00
#define HMC58X3_R_CONFB 0x01
#define HMC58X3_R_MODE 0x02
#define HMC58X3_R_XM 0x03
#define HMC58X3_R_XL 0x04

#define HMC58X3_R_YM 0x07  //!< Register address for YM.
#define HMC58X3_R_YL 0x08  //!< Register address for YL.
#define HMC58X3_R_ZM 0x05  //!< Register address for ZM.
#define HMC58X3_R_ZL 0x06  //!< Register address for ZL.

#define HMC58X3_R_STATUS 0x09
#define HMC58X3_R_IDA 0x10
#define HMC58X3_R_IDB 0x11
#define HMC58X3_R_IDC 0x12
void HMC5883_WriteReg(u8 reg_add,u8 reg_dat);
//void HMC58X3_getID(char id[3]);	//读芯片ID
void Init_HMC5883L(void);
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //读ADC
void HMC58X3_mgetValues(float *arry); //IMU 专用的读取磁力计值
//void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z);
//void Multiple_Read_HMC5883L(void);
#endif

//------------------End of File----------------------------
