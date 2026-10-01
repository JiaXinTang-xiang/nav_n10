#include "stm32f10x.h"                  // Device header

#ifndef __PWM_H
#define __PWM_H

void PWM_Z_Init(void);//左电机PWM初始化 最大值:100
void PWM_Z_SetCompare3(uint16_t Compare);//左电机PWM赋值 最大值:100
void PWM_Y_Init(void);//右电机PWM初始化 最大值:100
void PWM_Y_SetCompare4(uint16_t Compare);//右电机PWM赋值 最大值:100

#endif
