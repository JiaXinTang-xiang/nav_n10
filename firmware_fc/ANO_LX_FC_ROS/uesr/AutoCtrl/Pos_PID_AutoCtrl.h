#ifndef _POS_PID_AUTOCTRL_H_
#define	_POS_PID_AUTOCTRL_H_

//头文件
#include "stm32f4xx.h"
#include "Ano_Pid.h"



int Limit_max_int(int input,int max);
void keep_dis_x(double KP,double KI,double KD,int32_t dis_x_target,int32_t dis_x,int32_t max);//x轴环
void keep_dis_y(double KP,double KI,double KD,int32_t dis_y_target,int32_t dis_y,int32_t max);//y轴环
void PID1(void);


typedef struct
{
	s16 LocExpX;
	s16 LocExpY;
	u8 TurnOn_Flag;
	u8 halt_Flag;
}_fc_loc_exp_st;


extern _fc_loc_exp_st fc_loc_exp;

//void Loc_PID_Init(void);
//void FC_loc_PID(float dt);
//void FC_Loc1_PID(float dt);
//void FC_Loc2_PID(float dt);

#endif

