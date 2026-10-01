/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_PID.c
 * 描述    ：PID函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "Ano_Pid.h"
#include "Ano_Math.h"
/*

不难看出匿名使用的并不是大家推崇的增量式PID，而是位置式PID

简单列下关于位置式PID和增量式PID的公式

//位置式PID（匿名使用）
ux = kp * ex + ki * ex_sum + kd * (ex - ex_prior);

kp：比例系数 
P是比例控制 I 是积分控制  D是微分控制
积分控制是滞后的，是将实际值推向目标值，微分控制是超前的

//增量式PID
dux = kp(ex - ex_prior) + ki * ex + kd * (ex - 2 * ex_prior + ex_prior_prior);

ux = dux + ux;


*/


float PID_calculate(        			float dT_s,          //周期（单位：秒）
										float in_ff,				  //前馈值
										float expect,				  //期望值（设定值）
										float feedback,			  //反馈值（）
										_PID_arg_st *pid_arg, 	//PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,     	//积分误差限幅
										float inte_lim			  //integration limit，积分限幅									
										 )	
{
	float differential,hz;
	hz = safe_div(1.0f,dT_s,0);//安全除法 输入值分别为分子、分母、安全值
	
//	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);

  
  //如何正确理解微分定义？  dx/dt = lim  [x(t+dt) - x(t)]/dt  （其中在lim下面还有条件 dt->0）
	//很明显，下面使用的就是微分定义，同时为了加快运算速度，乘法确实比除法运算速度快，所以把除以dt->乘以hz
	
	
	//期望微分 = （期望 - 上次期望） * 频率	 	 	其实就是下面的骚操作版
	//期望微分 = （期望 - 上次期望） / 周期 （其中周期要足够小才能表示微分的定义，事实上在其他地方代值都是0.001s，完全足够）
	
	pid_val->exp_d = (expect - pid_val->exp_old) *hz; //期望的差值
	
	if(pid_arg->fb_d_mode == 0)
	{
    //反馈微分 = （反馈 - 上次反馈） * 频率  
		pid_val->fb_d = (feedback - pid_val->feedback_old) *hz; //反馈的差值
	}
	else
	{
    //事实上代码不可能走到这里
		pid_val->fb_d = pid_val->fb_d_ex;
	}
	//事实上前面求那么多都是为了后面这一步，至于为什么这样做我也不是很清楚
	//微分 = 期望微分系数 * 期望微分值 - 反馈微分系数 * 反馈微分值
	//微分搞定	
	differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);
  
	//积分累加并进行积分限幅
	//pid_val->err_i 表示总积分
	//pid_val->err 表示误差 
	//最终得出积分 积分搞定
	
   //误差 = 期望 - 反馈
	 pid_val->err = (expect - feedback);	

	//积分总和并且限幅
	pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;//)*T;//+ differential/pid_arg->kp
	//pid_val->err_i += pid_arg->ki *(pid_val->err )*T;//)*T;//+ pid_arg->k_pre_d *pid_val->feedback_d
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	
	//PID主体计算，调整电机维持姿态全靠它了
      pid_val->out = pid_arg->k_ff *in_ff      //前馈比例*前馈值
	    + pid_arg->kp *pid_val->err  //比例
			+	differential                //微分
//	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_val->err_i;             //积分
	
  //保存副本
	pid_val->feedback_old = feedback;
	pid_val->exp_old = expect;
	
  //最重要的就是得出这个输出值
	return (pid_val->out);
}





/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


