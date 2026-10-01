#include "Height_PID_AutoCtrl.h"
#include "Pos_PID_AutoCtrl.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"
#include "ANO_LX.h"


/*高度pid开启标志位宏定义*/
#define TurnOn_Flag (fc_loc_exp.halt_Flag)

//高度控制结构体
_halt_ctrl_st Fc_Height;
//高度控制参数
_PID_arg_st Height_arg;
//高度控制数据
_PID_val_st Height_val;

int height;


/*
飞控高度位置控制参数初始化
使用单环控制即可
*/
void Height_PID_Init(void)
{
	Height_arg.kp = 1.5;  //比例系数
	Height_arg.ki = 0.02f;   //误差系数
	Height_arg.kd_ex = 0.05f;  
	Height_arg.kd_fb =0.25f;   //反馈误差
	Height_arg.k_ff = 0.00f;	
	
}

/*
飞控搞定位置控制
*/
void Fc_Height_PID(float dt,float exp)
{
	//可以自行添加判断条件
	if(TurnOn_Flag)//开启高度控制pid标志位，使用宏定义，修改可以在上面的宏定义进行修改
	{
		Fc_Height.exp = exp;//期望高度，通过函数形参传递
		Fc_Height.fb = ano_of.of_alt_cm;//反馈高度，采用匿名光流传递的激光测距高度，可以自行修改为其他传感器的数据
		//PID控制函数调用
		PID_calculate(         
								dt*1e-3f,         	//周期（单位：秒）
								0,               	//前馈值
								Fc_Height.exp,   	//期望值（设定值）
								Fc_Height.fb,    	//反馈值（）
								&Height_arg,     	//PID参数结构体
								&Height_val,     	//PID数据结构体
								50,              	//积分误差限幅
								20 		 			//integration limit，积分限幅
					);
	
		Fc_Height.out  = Height_val.out;//pid计算输出的值对结构体进行赋值
	}
	//不使用PID控制时，输出进行赋值0
	else
	{
		Fc_Height.out = 0;
	}
	
	fc.acc_z = LIMIT(Fc_Height.out,-30.0f,30.0f);//输出进行限幅
}

