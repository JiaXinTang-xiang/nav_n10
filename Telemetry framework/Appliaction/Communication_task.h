#ifndef COMMUNICATION_TASK_H
#define COMMUNICATION_TASK_H

#include "bsp.h"

#define VISION_FRAME_HEAD          0x12U
#define VISION_FRAME_TAIL          0x5BU
#define VISION_MAX_DIGITS          4U
#define VISION_FRAME_SIZE          8U


typedef struct
{
    uint8_t Digit_Count;
    uint8_t Digits[VISION_MAX_DIGITS];
    uint8_t Result_Ready;

    uint32_t Byte_Count;
    uint32_t Frame_Count;
    uint32_t Error_Count;
} Struct_Vision_Data;

extern Struct_Vision_Data Vision_Data;
extern uint8_t Vision_Rx_Digits[VISION_MAX_DIGITS];
uint8_t Vision_GetResultFlag(void);
void UART_Vision_Call_Back(uint8_t *Buffer, uint16_t Length);

void UART_WireLess_Call_Back(uint8_t *Buffer, uint16_t Length);

#endif

