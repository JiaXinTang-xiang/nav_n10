#ifndef __MY_UART_H
#define __MY_UART_H

#include "stdint.h"


extern int STM32_Flag;
extern int K230_Flag;
extern uint8_t length;
void Recive_K230data(uint8_t data);
void Send_Data_To_32(uint8_t* send_data, uint8_t data_len);


//extern uint8_t _datatemp[50];
//extern int8_t land_flag;
//extern uint8_t datatemp[7];

#endif
