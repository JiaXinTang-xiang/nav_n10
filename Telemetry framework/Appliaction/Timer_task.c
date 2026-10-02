#include "Timer_task.h"
#include <stdbool.h>

/**
 * @brief  HAL 定时器溢出回调函数
 * @note   由 HAL 库在中断中自动调用, 需根据 htim->Instance 判断来源
 *         TIM6:  用于 IMU 姿态解算时间基准
		   周期 100us / 10khz
 * @param  htim  触发中断的定时器句柄
 */
volatile uint32_t nowtime;
extern float ypr[];
extern bool init_finished;

float target_angle;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t count0;
	static uint32_t count1;
	static uint32_t count2;
	
    if (htim->Instance == TIM6 && init_finished == 1)
    {
		nowtime++;
		count0++;
		count1++;
		count2++;
		
		if(nowtime%1000==0) LED1_toggle();	// ICM45686时钟基准
		     
		if(count0 >= 10)	// 1000hz任务
		{
			count0=0;
			Key_Tick();

		}
		if(count1 >= 50)	// 200hz任务
		{
			count1=0;
			Chassis_Update();	// 编码器读取 + 速度环 + 里程计 (5ms 周期)
	
		}
		if(count2 >= 200)		// 50hz任务
		{
			count2=0;
			LED1_toggle();
			Host_SendOdom();	// 每 20ms 发一帧 0xCC 里程计给上位机
		}
    }
}

