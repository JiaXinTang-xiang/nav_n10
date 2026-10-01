//////////////////////////////////////////////////////////////////////
/*
获取数据
*/
//////////////////////////////////////////////////////////////////////

#include "get_data.h"
#include "ANO_LX.h"
#include "stm32f4xx.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "mode.h"

double rol,pit,yaw,yaw_fix,yaw_zore,yaw_part,yaw_last,yaw_round;//imu输出的欧拉角，单位度
//float rol,pit,yaw,;

void get_data(void)//获取各种数据
{
//数据来自ANO_LX.h凌霄接收IMU数据
	rol = (fc_att.st_data.rol_x100)/100.0;
	pit = (fc_att.st_data.pit_x100)/100.0;
	
	yaw_part = fc_att.st_data.yaw_x100;
	if((yaw_last>9000)&&(yaw_part<-9000)) yaw_round++;
	if((yaw_last<-9000)&&(yaw_part>9000)) yaw_round--;
	yaw = yaw_round*360 + yaw_part/100.0;
	yaw_last=yaw_part;
	yaw_fix=yaw-yaw_zore;
	////////////////////////////////////////////////////

}


