#include "stm32f10x.h"                  // Device header
#include "fmq.h"

/**
  * 函    数：蜂鸣器初始化
  * 参    数：无
  * 返 回 值：无
  */
void fmq_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PB4引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOB, GPIO_Pin_4);				//设置PB4
}

/**
  * 函    数：蜂鸣器开启
  * 参    数：无
  * 返 回 值：无
  */
void fmq_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);		//设置PB4引脚为低电平
}

/**
  * 函    数：蜂鸣器
  * 参    数：无
  * 返 回 值：无
  */
void fmq_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_4);		//设置PB4引脚为高电平
}

