#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint8_t Key_Num;
/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
/* 1. 开启备份域访问（PC14/PC15 默认与 LSE 晶振复用，需要先关闭相关功能） */
RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
PWR_BackupAccessCmd(ENABLE);           // 允许修改备份域
BKP_TamperPinCmd(DISABLE);             // 关闭侵入检测功能
BKP_RTCOutputConfig(BKP_RTCOutputSource_None); // 禁用 RTC 输出

/* 2. 开启 GPIOC 时钟 */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

/* 3. GPIO 初始化（PC14 和 PC15 设置为上拉输入） */
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 上拉输入
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下
  * 注意事项：此函数是阻塞式操作，当按键按住不放时，函数会卡住，直到按键松手
  */
uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if(Key_Num)
	{
	Temp = Key_Num;
	Key_Num = 0;
	return Temp;
	}
	return 0;

}


uint8_t Key_GetState()
{
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14) == 0)
	{
		return 1;
	}
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15) == 0)
	{
		return 2;
	}
		return 0;
}

void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState;
	static uint8_t PrevState;

	
	Count ++;
	if(Count>=1)
	{
		Count=0;
		PrevState = CurrState;
		CurrState = Key_GetState();
		if(CurrState == 0 && PrevState != 0)
		{
			Key_Num = PrevState;
		}
		
	}
}

