#ifndef __SERIAL62_H
#define __SERIAL62_H
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#define  fp32 float
 
extern uint8_t Serial_RxPacket[];
extern uint8_t Serial_RxPacket2[];
extern uint8_t Serial_RxPacket3[];

typedef struct {
 fp32 Roll;
 fp32 Roll_2;
 fp32 Pitch;
 fp32 Pitch_2;
 fp32 Yaw;
 fp32 Yaw_2; 
}angle;


typedef struct {
 fp32 Wx;
 fp32 Wx_2;
 fp32 Wy;
 fp32 Wy_2;
 fp32 Wz;
 fp32 Wz_2; 
}angacc;


	typedef struct {
 fp32 Ax;
 fp32 Ax_2;
 fp32 Ay;
 fp32 Ay_2;
 fp32 Az;
 fp32 Az_2; 
}lineacc; 


extern angle Xreadangle;       // 假设 angleread 是某种类型
extern angacc Xreadangacc;
extern lineacc Xreadlineacc;
//我这里命名有点乱，可以改一下

void JY62_Get_Angle(void);
void JY62_Get_Angacc(void);
void JY62_Get_Lineacc(void);	
void JY62_Get_All(void);


void Serial62_Init(void);
void Serial62_SendByte(uint8_t Byte);
void Serial62_SendArray(uint8_t *Array, uint16_t Length);
void Serial62_SendString(char *String);
void Serial62_SendNumber(uint32_t Number, uint8_t Length);
void Serial62_Printf(char *format, ...);
uint8_t Serial62_GetRxFlag(void);
uint8_t Serial62_GetRxData(void);


//void SendData(char x,char y);
int data_test(uint8_t data[]);

#endif
//2026.04.27  hzx
