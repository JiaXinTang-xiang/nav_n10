#ifndef __TIME5_8_H
#define __TIME5_8_H

#include "stm32f4xx.h"

#define PWM_CH8		8
#define PWM_CH7		7

#define PWM_CH6		6
#define PWM_CH5		5

void TIM5_Init(u16 arr,u16 psc);  
void Tim5_Servo180_On_Off(u8 Angle,u8 pwm_chx);
void Tim5_Servo360_On_Off(u8 mode,u8 pwm_chx);

void TIM8_Init(u16 arr,u16 psc);
void Tim8_Servo180_On_Off(u8 Angle,u8 pwm_chx);
void Tim8_Servo360_On_Off(u8 mode,u8 pwm_chx);


#endif

