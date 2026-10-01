#include "stm32f10x.h"
#include "PWM.h"

static void PWM_TIM3_BaseInit(void)
{
    static uint8_t initialized = 0;
    if (initialized) return;          // 已经初始化过，直接返回
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 启用 TIM3 部分重映射 (CH3->PB0, CH4->PB1)
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;        // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;      // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    
    initialized = 1;
}

void PWM_Z_Init(void)   // 左电机PWM初始化(PB0)
{
    PWM_TIM3_BaseInit();   // 确保时基已配置
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // PB0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;        // CCR
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    
    TIM_Cmd(TIM3, ENABLE);
}

void PWM_Z_SetCompare3(uint16_t Compare)
{
    TIM_SetCompare3(TIM3, Compare);
}

void PWM_Y_Init(void)   // 右电机PWM初始化(PB1)
{
    PWM_TIM3_BaseInit();   // 确保时基已配置
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // PB1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;        // CCR
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    
    TIM_Cmd(TIM3, ENABLE);
}

void PWM_Y_SetCompare4(uint16_t Compare)
{
    TIM_SetCompare4(TIM3, Compare);
}
