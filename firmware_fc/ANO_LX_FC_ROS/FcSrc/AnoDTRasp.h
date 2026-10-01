#ifndef __ANO_DT_RASP_H
#define __ANO_DT_RASP_H
//==引用
#include "stm32f4xx.h"

//通信方式定义
#define LinkType_ToRasp		0x01
#define LinkType_ToIMU		0x02
//发送数据帧时临时存储数据帧信息的结构体
typedef struct
{
	u8 _isUsed;
	u8 _linkType;			
}__attribute__ ((__packed__)) myTPTv7TxStruct;

typedef struct
{
	u8 head;
	u8 addr;
	u8 ID;
	u8 dataLen;
	u8 dataBuf[256];
	u8 sc1;
	u8 sc2;
}__attribute__ ((__packed__)) myTAnoPTv7Struct;	 

typedef union 
{
	myTAnoPTv7Struct frame;
	u8 rawBytes[sizeof(myTAnoPTv7Struct)];
}__attribute__ ((__packed__)) myTAnoPTv7Frame;

//用于发送的结构体，data保存帧内容，cmd保存发送参数
typedef struct
{
	myTAnoPTv7Frame data;		
	myTPTv7TxStruct cmd; 			
}__attribute__ ((__packed__)) myTPTv7TxFrame;

typedef struct
{
	u8 work_sta;
	u8 getRosDataFlag;//标志位
	s16 tacc[3];//t265加速度
	s16 tloc[3];//t265位置
	s16 loc[3];//激光雷达建图定位
	
    s16 acc[3];//ros控制无人机速度
    s16 yaw;//ros控制无人机平航
}ROS_DATA_ST;
extern ROS_DATA_ST rosData;


void AnoDTRaspRunTask1Ms(void);
void AnoDTRaspRecvOneByte(u8 data);

void AnoPTv7SendUserData(u8 d1,u8 d2,u8 d3,s16 d4,s16 d5,s16 d6);

void AnoPTv7Send_ShutDown(void);
void AnoPTv7Send_Restart(void);
void AnoPTv7SendBuf(const uint8_t ID,const uint8_t* buf, const uint8_t len);

#endif
