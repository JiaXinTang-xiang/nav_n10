#include "stm32f10x.h" // Device header
#include "PWM.h"
#include "Motor.h"

void Motor_Z_Init(void)
{ // 左电机初始化(PA11,PA12)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    PWM_Z_Init();
}

void Motor_Z_SetSpeed(int16_t Speed)
{ // 左电机设置速度
    if (Speed >= 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_12);
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        PWM_Z_SetCompare3(Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_12);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        PWM_Z_SetCompare3(-Speed);
    }
}

void Motor_Y_Init(void)
{ // 右电机初始化(PB12,PB13)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PWM_Y_Init();
}

void Motor_Y_SetSpeed(int16_t Speed)
{ // 右电机设置速度
    if (Speed >= 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        PWM_Y_SetCompare4(Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        PWM_Y_SetCompare4(-Speed);
    }
}
