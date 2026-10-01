#include "Pos_PID_AutoCtrl.h"
#include "Ano_Math.h"
#include "AnoDTRasp.h"
#include "ANO_LX.h"
#include "User_Task.h"

_fc_loc_exp_st fc_loc_exp;

//double KP_radar=0.75,KI_radar=0.0,KD_radar=7.32,integrate_max=30;//激光雷达pid参数 （踢踢调）

double KP_radar=0.80,KI_radar=0.0196,KD_radar=6.9,integrate_max=30;//激光雷达pid参数 

//double KP_radar=0.75,KI_radar=0.0196,KD_radar=7.0,integrate_max=30;//激光雷达pid参数 

#define FILTER_WINDOW_SIZE 6  // 滑动窗口大小，可根据需要调整


// 滑动平均滤波器结构体
typedef struct {
    int32_t buffer[FILTER_WINDOW_SIZE];  // 数据缓冲区
    int index;                           // 当前索引
    int count;                           // 数据计数
    int32_t sum;                         // 总和，用于快速计算平均值
} MovingAverageFilter;

// 初始化滤波器
void initFilter(MovingAverageFilter *filter) {
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0;
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        filter->buffer[i] = 0;
    }
}

// 添加新数据并获取滤波后的值
int32_t filterData(MovingAverageFilter *filter, int32_t newData) {
    // 减去即将被替换的数据
    if (filter->count == FILTER_WINDOW_SIZE) {
        filter->sum -= filter->buffer[filter->index];
    } else {
        filter->count++;
    }
    
    // 添加新数据
    filter->buffer[filter->index] = newData;
    filter->sum += newData;
    
    // 更新索引
    filter->index = (filter->index + 1) % FILTER_WINDOW_SIZE;
    
    // 返回平均值
    return filter->sum / filter->count;
}

int Limit_max_int(int input,int max)//限幅函数
{
	if(input > max)
	{
		input = max;
	}
	else if(input < -max)
	{
		input = -max;
	}
	return input;
}


void keep_dis_x(double KP,double KI,double KD,int32_t dis_x_target,int32_t dis_x,int32_t max)//x轴环
{
	int32_t error,KP_out,KI_out,KD_out,total_out;
	static int32_t dis_integrate=0,error_last=0;
	
	error = dis_x_target-dis_x;

//	if(error > 20)  error = 20;		////自己加的，不确定有没有用
//  if(error < -20) error = -20;	
	
	KP_out = KP*error;
	
	KD_out = (error-error_last)*KD;
	if(error_last!=error) error_last = error;
	
	 dis_integrate += error;
    // 抗积分饱和：限制积分累积值
    if (dis_integrate > integrate_max) 
        dis_integrate = integrate_max;
    else if (dis_integrate < -integrate_max)
        dis_integrate = -integrate_max;
     KI_out = KI * dis_integrate;

	
	total_out = KP_out+KI_out+KD_out;
	total_out = Limit_max_int(total_out,max);
	
	fc.acc_x = total_out;
}

void keep_dis_y(double KP,double KI,double KD,int32_t dis_y_target,int32_t dis_y,int32_t max)//y轴环
{
	int32_t error,KP_out,KI_out,KD_out,total_out;
	static int32_t dis_integrate=0,error_last=0;
	
	error = dis_y_target-dis_y;
	
//	if(error > 20)  error = 20;		////自己加的，不确定有没有用
//  if(error < -20) error = -20;
	
	KP_out = KP*error;
	
	KD_out = (error-error_last)*KD;
	if(error_last!=error) error_last = error;
	
	 dis_integrate += error;
    // 抗积分饱和：限制积分累积值
    if (dis_integrate > integrate_max) 
        dis_integrate = integrate_max;
    else if (dis_integrate < -integrate_max)
        dis_integrate = -integrate_max;
     KI_out = KI * dis_integrate;
	
	total_out = KP_out+KI_out+KD_out;
	total_out = Limit_max_int(total_out,max);
	
	 fc.acc_y = total_out;
}

// 定义x和y轴的滤波器
static MovingAverageFilter xFilter, yFilter;
static int filterInitialized = 0;

void PID1(void)//遥控输入值的PID运算
{
    // 初始化滤波器（只执行一次）
    if (!filterInitialized) {
        initFilter(&xFilter);
        initFilter(&yFilter);
        filterInitialized = 1;
    }
    
    // 对雷达数据进行滤波处理-position_x-position_y
    int32_t filtered_x = filterData(&xFilter, -rosData.loc[0]);
    int32_t filtered_y = filterData(&yFilter, -rosData.loc[1]);
		  
	if(fc_loc_exp.TurnOn_Flag)
	{	
        // 使用滤波后的数据进行PID控制
		keep_dis_x(KP_radar,KI_radar,KD_radar, fc_loc_exp.LocExpX, filtered_x, 10);////这里的max=10，相当于限制速度了
		keep_dis_y(KP_radar,KI_radar,KD_radar, fc_loc_exp.LocExpY, filtered_y, 10);                       
	}	
}



////位置环PID参数初始化
//void Loc_PID_Init(void)
//{
//	Loc_arg1[X].kp = 1.0f;  //比例系数
//	Loc_arg1[X].ki = 0.0f;   //误差系数
//	Loc_arg1[X].kd_ex = 0.0f ;  
//	Loc_arg1[X].kd_fb = 0.05f;   //反馈误差
//	Loc_arg1[X].k_ff = 0.00f;
//	Loc_arg1[Y] = Loc_arg1[X]; //使用同一参数
//	
//	Loc_arg2[X].kp = 1.0f;  //比例系数
//	Loc_arg2[X].ki = 0.0f;   //误差系数
//	Loc_arg2[X].kd_ex = 1.5f ;  
//	Loc_arg2[X].kd_fb = 0.05f;   //反馈误差
//	Loc_arg2[X].k_ff = 0.00f;
//	Loc_arg2[Y] = Loc_arg2[X]; //使用同一参数
//}

///*
//调用该函数即可进行无人机的位置控制PID
//*/
//void FC_loc_PID(float dt)
//{
//	FC_Loc1_PID(dt);
//	FC_Loc2_PID(dt);
//}

////位置控制PID位置环
//void FC_Loc1_PID(float dt)
//{

//	if(Loc1_TurnOn_Flag == 1)//开启位置PID控制
//	{
//     if(1)//接收到ros发送数据 //添加其他判断条件进行PID控制
//		{
//			Fc_Loc1.exp[X] 	= Loc1_expX;//位置期望x数据
//			Fc_Loc1.exp[Y] 	= Loc1_expY;//位置期望y数据
//			
//			Fc_Loc1.fb[X]	= Loc1_fbX;//位置反馈x数据
//			Fc_Loc1.fb[Y]	= Loc1_fbY;//位置反馈y数据
//				
//			for(u8 i=0;i<2;i++)
//			{
//				//PID控制函数调用
//				PID_calculate(         
//										dt*1e-3f,         	//周期（单位：秒）
//										0,               	//前馈值
//										Fc_Loc1.exp[i],   	//期望值（设定值）
//										Fc_Loc1.fb[i],    	//反馈值（）
//										&Loc_arg1[i],     	//PID参数结构体
//										&Loc_val1[i],     	//PID数据结构体
//										50,              	//积分误差限幅
//										20 		 			//integration limit，积分限幅
//							);
//			}
//			Fc_Loc1.out[X] = Loc_val1[X].out;
//			Fc_Loc1.out[Y] = Loc_val1[Y].out;
//			Fc_Loc2.TurnOn_Flag = 1; //位置环执行成功后，速度环才能执行
//		}
//		else
//		{
//			Fc_Loc1.out[X] = Fc_Loc1.out[Y] = 0;
//			Fc_Loc2.TurnOn_Flag = 0;
//			
//		}
//	}
//	else
//	{
//		Fc_Loc1.out[X] = Fc_Loc1.out[Y] = 0;
//		Fc_Loc2.TurnOn_Flag = 0;
//	}
//	
//}

////位置控制PID速度环
//void FC_Loc2_PID(float dt)
//{
//	if(Fc_Loc2.TurnOn_Flag == 1)//开启位置速度PID控制
//	{
//			Fc_Loc2.exp[X] 	= LIMIT(Fc_Loc1.out[X],-20.0f,20.0f);;//位置速度期望x数据
//			Fc_Loc2.exp[Y] 	= LIMIT(Fc_Loc1.out[Y],-20.0f,20.0f);;//位置速度期望y数据
//			
//			Fc_Loc2.fb[X]	= Loc2_fbX;//位置速度反馈x数据
//			Fc_Loc2.fb[Y]	= Loc2_fbY;//位置速度反馈y数据
//				
//			for(u8 i=0;i<2;i++)
//			{
//				//PID控制函数调用
//				PID_calculate(         
//										dt*1e-3f,         	//周期（单位：秒）
//										0,               	//前馈值
//										Fc_Loc2.exp[i],   	//期望值（设定值）
//										Fc_Loc2.fb[i],    	//反馈值（）
//										&Loc_arg2[i],     	//PID参数结构体
//										&Loc_val2[i],     	//PID数据结构体
//										50,              	//积分误差限幅
//										10 		 			//integration limit，积分限幅
//							);
//			}
//			Fc_Loc2.out[X] = Loc_val2[X].out;
//			Fc_Loc2.out[Y] = Loc_val2[Y].out;
//	}
//	else
//	{
//		Fc_Loc2.out[X] = Fc_Loc2.out[Y] = 0;

//	}
//	
//	fc.acc_x = LIMIT(Fc_Loc2.out[X],-20.0f,20.0f);//限幅输出
//	fc.acc_y = LIMIT(Fc_Loc2.out[Y],-20.0f,20.0f);//限幅输出
//}



