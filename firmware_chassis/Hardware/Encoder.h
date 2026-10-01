#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void Encoder_Z_Init(void);     // 左编码器初始化 (PA0/PA1, TIM2)
int16_t Encoder_Z_Get(void);   // 左编码器增量 (读取后清零)
void Encoder_Y_Init(void);     // 右编码器初始化 (PB6/PB7, TIM4)
int16_t Encoder_Y_Get(void);   // 右编码器增量 (读取后清零)

#endif
