#ifndef _USART_H
#define _USART_H

#include "SysConfig.h"

void DrvUart1Init(u32 br_num);
void Usart1_IRQ(void);
void DrvUart1SendBuf(unsigned char *DataToSend, u8 data_num);
void DrvUart1SendInt16Buf(int16_t *values, u8 num);
void DrvUart1SendDouble(double value);
void DrvUart1SendDoubles(double *values, u8 num);// ·¢ËÍ double Êý×é
extern void my_uart1_Receive(uint8_t data);


void DrvUart2Init(u32 br_num);
void Usart2_IRQ(void);
void DrvUart2SendBuf(unsigned char *DataToSend, u16 data_num);
extern void my_uart2_Receive(uint8_t data);

void DrvUart3Init(u32 br_num);
void Usart3_IRQ(void);
void DrvUart3SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart4Init(u32 br_num);
void Uart4_IRQ(void);
void DrvUart4SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart5Init(u32 br_num);
void Uart5_IRQ(void);
void DrvUart5SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUartDataCheck(void);
#endif
