#ifndef __ANO_PID_H
#define __ANO_PID_H
//#include "Ano_FcData.h"
#include "stm32f4xx.h"
/*=====================================================================================================================
						 *****
=====================================================================================================================*/
enum
{
	X = 0,
	Y,
	Z,
	XYZ
};


typedef struct
{
	u8 fb_d_mode;
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd_ex;		//微分系数  期望微分系数
	float kd_fb;    //previous_d 微分先行   反馈微分系数
//	float inc_hz;  //不完全微分低通系数
//	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 

}_PID_arg_st;




typedef struct
{
	float err;       //误差
	float exp_old;    //上次期望
	float feedback_old; //上次反馈
	
	float fb_d;     //反馈微分值fb反馈 d 数据
	float fb_d_ex;//ex期望
	float exp_d;    //期望微分值
//	float err_d_lpf;
	float err_i;    //误差总积分
	float ff;//
	float pre_d;//先前值

	float out;    //高度输出
}_PID_val_st;

float PID_calculate( float T,            //周期
										float in_ff,				//前馈
										float expect,				//期望值（设定值）
										float feedback,			//反馈值
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,
										float inte_lim			//integration limit，积分限幅
										   );			//输出


										
#endif

