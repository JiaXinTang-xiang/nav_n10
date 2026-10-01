//////////////////////////////////////////////////////////////////////
/*
模式选择
*/
//////////////////////////////////////////////////////////////////////

#include "mode.h"
#include "Pos_PID_AutoCtrl.h"
#include "Height_PID_AutoCtrl.h"

int my_mode;//飞行模式
//int dis_target_ref,dis_group[2][20]={{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19},{1,0,2,0,0,0,2,0,0,0,2,3,0}};

void mode_select(int mode)//选择飞行模式，改变标志位
{
	switch(mode)
	{
		case 1:{fc_loc_exp.halt_Flag = 0;fc_loc_exp.TurnOn_Flag =0;}break;//测试
		case 2:{fc_loc_exp.halt_Flag = 0;fc_loc_exp.TurnOn_Flag =1;}break;				
    case 3:{fc_loc_exp.halt_Flag = 1;fc_loc_exp.TurnOn_Flag =0;}break;
		case 4:{fc_loc_exp.halt_Flag = 1;fc_loc_exp.TurnOn_Flag =1;}break;//激光雷达
	}
}


void dw_select(int dw)
{
	switch(dw)
	{
		//起飞点为坐标原点		
		case 0:{ fc_loc_exp.LocExpX=0; fc_loc_exp.LocExpY=0; }break;
		case 1:{ fc_loc_exp.LocExpX=160;fc_loc_exp.LocExpY=0;}break;
		case 2:{ fc_loc_exp.LocExpX=156;fc_loc_exp.LocExpY=-74;}break;
		case 3:{ fc_loc_exp.LocExpX=160;fc_loc_exp.LocExpY=-160;}break;
		case 4:{ fc_loc_exp.LocExpX=0;fc_loc_exp.LocExpY=-160;}break;
	}
}






