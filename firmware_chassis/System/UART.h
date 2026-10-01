#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"                  // Device header
#include <stdio.h>

void UART1_Init(void);
void UART1_SendByte(uint8_t Byte);
void UART1_SendArray(uint8_t *Array, uint16_t Length);
void UART1_SendString(char *String);
void UART1_SendNumber(uint32_t Number, uint8_t Length);
void UART1_Printf(char *format, ...);

uint8_t UART1_GetRxFlag(void);
uint8_t UART1_GetRxData(void);


//uint8_t RxData;			//定义用于接收串口数据的变量

//int main(void)
//{
//	/*模块初始化*/
//	OLED_Init();		//OLED初始化
//	
//	/*显示静态字符串*/
//	OLED_ShowString(1, 1, "RxData:");
//	
//	/*串口初始化*/
//	UART1_Init();		//串口初始化
//	
//	while (1)
//	{
//		if (UART1_GetRxFlag() == 1)			//检查串口接收数据的标志位
//		{
//			RxData = UART1_GetRxData();		//获取串口接收的数据
//			UART1B_SendByte(RxData);			//串口将收到的数据回传回去，用于测试
//			OLED_ShowHexNum(1, 8, RxData, 2);	//显示串口接收的数据
//		}
//	}
//}




#endif
