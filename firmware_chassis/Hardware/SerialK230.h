#ifndef __SERIALK230_H
#define __SERIALK230_H

#include <stdio.h>
#include "stm32f10x.h" // Device header

void Serial_Init_K(void);
void Serial_SendByte_K(uint8_t Byte);
void Serial_SendArray_K(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

extern char Serial_RxPacket_K[];
extern uint8_t Serial_RxFlag_K;

extern int k230_w;
extern int k230_h;
extern int k230_flag;

uint8_t Serial_GetRxFlag_K(void);
void Serial_GetData_K(void);

#endif
