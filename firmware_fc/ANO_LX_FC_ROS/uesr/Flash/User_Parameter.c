#include "User_Parameter.h"
#include "ANO_DT_LX.h"
#include  "Drv_w25qxx.h"
#include "Ano_Math.h"
#include "Height_PID_AutoCtrl.h"
#include "Pos_PID_AutoCtrl.h"


union Parameter Ano_Parame;
_parameter_state_st para_sta;

/*
参数数据复位
*/
void Parameter_DATA_Rest(void)
{
	/*添加数据，进行初始化*/
	
	LxStringSend(2,(char*)"Parameter DATA reset!");
}

static void Ano_Parame_Write(void)
{
	
	/*存储参数数据初始化*/
	//All_PID_Init();	//存储PID参数后，重新初始化PID	
	Ano_Parame.set.frist_init = SOFT_VER;	
	Flash_SectorErase ( 0x000000, 1 );							//擦除第一扇区
	Flash_SectorsWrite ( 0x000000, &Ano_Parame.byte[0], 1 );	//将参数写入第一扇区
	LxStringSend(2,(char*)"写入Flash成功");
}

void Ano_Parame_Read(void)
{
	Flash_SectorsRead ( 0x000000, &Ano_Parame.byte[0], 1 );		//读取第一扇区内的参数
	
	if(Ano_Parame.set.frist_init != SOFT_VER)	//内容没有被初始化，则进行参数初始化工作
	{		
		Parameter_DATA_Rest();
		Ano_Parame_Write();
	}
	LxStringSend(2,(char*)"读取Flash成功");
}

void Ano_Parame_Write_task(u16 dT_ms)
{
	//因为写入flash耗时较长，我们飞控做了一个特殊逻辑，在解锁后，是不进行参数写入的，此时会置一个需要写入标志位，等飞机降落锁定后，再写入参数，提升飞行安全性
	//为了避免连续更新两个参数，造成flash写入两次，我们飞控加入一个延时逻辑，参数改变后三秒，才进行写入操作，可以一次写入多项参数，降低flash擦写次数
	if(para_sta.save_en )				//允许存储
	{
		if(para_sta.save_trig == 1) 	//如果触发存储标记1
		{			
			para_sta.time_delay = 0;  	//计时复位
			para_sta.save_trig = 2;   	//触发存储标记2
		}
		
		if(para_sta.save_trig == 2) 	//如果触发存储标记2
		{
			if(para_sta.time_delay<3000) //计时小于3000ms
			{
				para_sta.time_delay += dT_ms; //计时
			}
			else
			{
								
				Ano_Parame_Write();      //执行存储
				LxStringSend(2,(char*)"Set save OK!");
				para_sta.save_trig = 0;  //存储标记复位
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}	
	}
	else
	{
		para_sta.time_delay = 0;
		para_sta.save_trig = 0;
	}
}

/*
参数初始化
*/
void Para_Data_Init(void)
{
	Ano_Parame_Read();
}

/*
PID参数初始化
*/
void All_PID_Init(void)
{
	//高度PID参数初始化
	Height_PID_Init();
	//位置PID参数初始化
	 PID1();
//	Loc_PID_Init();
}

/*
保存函数，调用只能在飞机没有起飞不或者解锁时
*/
void data_save(void)
{
	para_sta.save_en = 0;
	para_sta.save_trig = 1;
}







