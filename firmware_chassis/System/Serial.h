#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"

/* ========== 接收缓冲区 (取最大协议长度) ========== */
extern uint8_t Serial_RxPacket[];

/* ========== 视觉数据 (0xAA协议) ========== */
extern float center_x;
extern float center_y;
extern float area_value;

/* ========== 速度指令 (0xBB协议) ========== */
extern float cmd_v_linear;          // 线速度 (mm/s)
extern float cmd_v_angular;         // 角速度 (mrad/s)
extern volatile uint8_t cmd_vel_flag; // 收到新速度指令标志

/* ========== API ========== */
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);

uint8_t Serial_GetRxFlag(void);
void Serial_GetData(uint16_t *X, uint16_t *Y, uint16_t *Area);

uint8_t Serial_GetCmdVelFlag(void);
void Serial_SendOdometry(float x, float y, float theta);

#endif
