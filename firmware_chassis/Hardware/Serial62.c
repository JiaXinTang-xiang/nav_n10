#include "stm32f10x.h"                 
#include "Serial62.h"
#include <stdio.h>
#include <stdarg.h>
#include <OLED.h>
#include <string.h>
 uint8_t Serial_RxPacket1[9];
uint8_t Serial_RxPacket2[9]; 
uint8_t Serial_RxPacket3[9];//数据包

 uint8_t RCRSerial_RxPacket[9];
uint8_t RCRSerial_RxPacket2[9]; 
uint8_t RCRSerial_RxPacket3[9];//数据包
  	
  	
uint8_t Serial62_RxFlag;		//接受角度用的标志位
uint8_t Serial62_RxData;		//定义串口接收的数据变量 

angle Xreadangle;
angacc Xreadangacc;
lineacc	Xreadlineacc;

uint8_t data[6];  //测试数据包
extern float xf,xf_2; //接手数据包的角度 

void Serial62_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
}
 
/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial62_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}
 

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial62_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial62_SendByte(Array[i]);
	}
}
 

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial62_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial62_SendByte(String[i]);
	}
}
 
/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial62_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}
 
/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial62_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial62_SendByte(Number / Serial62_Pow(10, Length - i - 1) % 10 + '0');
	}
}
 

 

 
 
/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial62_GetRxFlag(void)
{
	if (Serial62_RxFlag == 1)
	{
		Serial62_RxFlag = 0;
		return 1;
	}
	return 0;
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */
uint8_t Serial62_GetRxData(void)
{
	return Serial62_RxData;			//返回接收的数据变量
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */

void USART3_IRQHandler(void) //陀螺仪用的
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	static uint8_t pRxPacket2 = 0;
	static uint8_t pRxPacket3 = 0;
	static u16 RCR53=0;
	static u16 RCR52=0;
	static	u16 RCR51=0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART3);
		
		if (RxState == 0)
		{    
			switch(RxData)
			{                 
				case 0x53:
					RxState = 1;
        			pRxPacket = 0;RCR53=0x53+0x55;break;
				case 0x52:
				    RxState = 3;
				    pRxPacket2 = 0;RCR52=0x52+0x55;break;
				case 0x51:
				    RxState = 5;
				    pRxPacket3 = 0;RCR51=0x51+0x55;break;
				default:break;
			}
		}
		else if (RxState != 0)
		{
			 switch(RxState)
			{
			    case 1:
					Serial_RxPacket1[pRxPacket] = RxData;
			     	pRxPacket ++;
				    if (pRxPacket < 9)
				    {RCR53+=RxData;}			
			        if (pRxPacket >= 9)
			          {   
				          RxState = 0;
						  if((RCR53&0xff)==RxData)
						  {memcpy(RCRSerial_RxPacket,  Serial_RxPacket1,9);}

						  Serial62_RxFlag = 1;
						  RCR53=0x53+0x55;
						 
			          }break;
			   case 3:
					Serial_RxPacket2[pRxPacket2] = RxData;
			     	pRxPacket2 ++;
				    if (pRxPacket2 < 9)
				    {RCR52+=RxData;}			
			        if (pRxPacket2 >= 9)
			          {   
				          RxState = 0;
						  if((RCR52&0xff)==RxData)
						  {memcpy(RCRSerial_RxPacket2,  Serial_RxPacket2,9);}

						  Serial62_RxFlag = 1;
						  RCR52=0x52+0x55;
						 
			          }break;
			    case 5:
					Serial_RxPacket3[pRxPacket3] = RxData;
			     	pRxPacket3 ++;
				    if (pRxPacket3 < 9)
				    {RCR51+=RxData;}			
			        if (pRxPacket3 >= 9)
			          {   
				          RxState = 0;
						  if((RCR51&0xff)==RxData)
						  {memcpy(RCRSerial_RxPacket3,  Serial_RxPacket3,9);}

						  Serial62_RxFlag = 1;
						  RCR51=0x51+0x55;
						 
			          }break;
			   default:break;
		        }
		   }
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}


void JY62_Get_Angle(void)	
{	Xreadangle.Pitch=((short)(RCRSerial_RxPacket[1]<<8)|RCRSerial_RxPacket[0])/32768.0*180;
	Xreadangle.Pitch_2=((RCRSerial_RxPacket[1]<<8)|RCRSerial_RxPacket[0])/32768.0*180;
	Xreadangle.Roll=((short)(RCRSerial_RxPacket[3]<<8)|RCRSerial_RxPacket[2])/32768.0*180;
	Xreadangle.Roll_2=((RCRSerial_RxPacket[3]<<8)|RCRSerial_RxPacket[2])/32768.0*180;
	Xreadangle.Yaw=((short)(RCRSerial_RxPacket[5]<<8)|RCRSerial_RxPacket[4])/32768.0*180;
	Xreadangle.Yaw_2=((RCRSerial_RxPacket[5]<<8)|RCRSerial_RxPacket[4])/32768.0*180;	
}

void JY62_Get_Angacc(void)	
{	Xreadangacc.Wx=((short)(RCRSerial_RxPacket2[1]<<8)|RCRSerial_RxPacket2[0])/32768.0*2000;
	Xreadangacc.Wx_2=((RCRSerial_RxPacket2[1]<<8)|RCRSerial_RxPacket2[0])/32768.0*2000;
	Xreadangacc.Wy=((short)(RCRSerial_RxPacket2[3]<<8)|RCRSerial_RxPacket2[2])/32768.0*2000;
	Xreadangacc.Wy_2=((RCRSerial_RxPacket2[3]<<8)|RCRSerial_RxPacket2[2])/32768.0*2000;
	Xreadangacc.Wz=((short)(RCRSerial_RxPacket2[5]<<8)|RCRSerial_RxPacket2[4])/32768.0*2000;
	Xreadangacc.Wz_2=((RCRSerial_RxPacket2[5]<<8)|RCRSerial_RxPacket2[4])/32768.0*2000;	
}

void JY62_Get_Lineacc(void)	
{	Xreadlineacc.Ax=((short)(RCRSerial_RxPacket3[1]<<8)|RCRSerial_RxPacket3[0])/32768.0*160;
	Xreadlineacc.Ax_2=((RCRSerial_RxPacket3[1]<<8)|RCRSerial_RxPacket3[0])/32768.0*16;
	Xreadlineacc.Ay=((short)(RCRSerial_RxPacket3[3]<<8)|RCRSerial_RxPacket3[2])/32768.0*160;
	Xreadlineacc.Ay_2=((RCRSerial_RxPacket3[3]<<8)|RCRSerial_RxPacket3[2])/32768.0*16;
	Xreadlineacc.Az=((short)(RCRSerial_RxPacket3[5]<<8)|RCRSerial_RxPacket3[4])/32768.0*160;
	Xreadlineacc.Az_2=((RCRSerial_RxPacket3[5]<<8)|RCRSerial_RxPacket3[4])/32768.0*16;	
}

void JY62_Get_All(void)
{
    JY62_Get_Angle();
    JY62_Get_Angacc();
    JY62_Get_Lineacc();
}
