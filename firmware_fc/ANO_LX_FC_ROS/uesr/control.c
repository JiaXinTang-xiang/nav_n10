/////////////////////////////////////////////////////////////////////
/*
底层控制
*/
//////////////////////////////////////////////////////////////////////

#include "control.h"
#include "stm32f4xx.h"                  // Device header

int control_flag;
int time,time_cnt_ms;//普通计时
unsigned int  mission_step,mission_step1,mission_step2,mission_step3,mission_step4;//步骤

void all_data_init(void)//步骤二进行全部数据的初始化
{
	time=0;
	time_cnt_ms=0;
}

void User_Task_Delay(int ms)//步骤中进行延时函数
{
	if(time<ms){time+=20;}
	else{time=0;mission_step++;}			
}

void User_Task_Delay1(int ms)//步骤中进行延时函数
{
	if(time<ms){time+=20;}
	else{time=0;mission_step1++;}			
}

void User_Task_Delay2(int ms)//步骤中进行延时函数
{
	if(time<ms){time+=20;}
	else{time=0;mission_step2++;}			
}

void User_Task_Delay3(int ms)//步骤中进行延时函数
{
	if(time<ms){time+=20;}
	else{time=0;mission_step3++;}			
}

void User_Task_Delay4(int ms)//步骤中进行延时函数
{
	if(time<ms){time+=20;}
	else{time=0;mission_step4++;}			
}




int square_wave(int amplitude)//阶跃函数，周期10s
{
	int out;
	if(time_cnt_ms/5000%2==0)out=amplitude;
	if(time_cnt_ms/5000%2==1)out=0;
	return out;
}

int triangualr_wave(int amplitude)//三角函数，周期10s
{
	int out;
	if(time_cnt_ms/5000%2==0)out=time_cnt_ms%5000/5000.0*amplitude;
	if(time_cnt_ms/5000%2==1)out=(1-time_cnt_ms%5000/5000.0)*amplitude;
	return out;
}


