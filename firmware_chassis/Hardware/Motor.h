#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/* TB6612 引脚定义
 * 左电机: AIN1=PA11, AIN2=PA12, PWMA=PB0(TIM3_CH3)
 * 右电机: BIN1=PB12, BIN2=PB13, PWMB=PB1(TIM3_CH4)
 */

void Motor_Z_Init(void);                  // 左电机初始化
void Motor_Z_SetSpeed(int16_t Speed);     // 左电机速度设置 (-100 ~ +100)
void Motor_Y_Init(void);                  // 右电机初始化
void Motor_Y_SetSpeed(int16_t Speed);     // 右电机速度设置 (-100 ~ +100)

#endif
