#include "Drv_AnoOf.h"

/*匿名光流版本选择 3--v3 4--v4*/
#define AnoOFChoose 4

_ano_of_st ano_of;
#define OF_TEMP_LEN  50
static uint8_t _datatemp[OF_TEMP_LEN];
static float check_time_ms[3];
void AnoOF_Check_State(float dT_s)
{
	u8 tmp[2];
	//连接检查
	if (check_time_ms[0] < 500)
	{
		check_time_ms[0]++;
		ano_of.link_sta = 1;
	}
	else
	{
		ano_of.link_sta = 0;
	}
	//数据检查1
	if (check_time_ms[1] < 500)
	{
		check_time_ms[1]++;
		tmp[0] = 1;
	}
	else
	{
		tmp[0] = 0;
	}
	//数据检查2
	if (check_time_ms[2] < 500)
	{
		check_time_ms[2]++;
		tmp[1] = 1;
	}
	else
	{
		tmp[1] = 0;
	}
	//设置工作状态
	if (tmp[0] && tmp[1])
	{
		ano_of.work_sta = 1;
	}
	else
	{
		ano_of.work_sta = 0;
	}
}

/*
该函数通过宏定义AnoOFChoose进行匿名光流版本数据获取选择
*/
void AnoOF_GetOneByteChoose(uint8_t data)
{
#if (AnoOFChoose == 3) 
		AnoOF_GetOneByte_v3(data);//老版本红色V3版本，协议使用V6，不支持6面校准，安装时USB朝飞机左侧
#else
		AnoOF_GetOneByte(data);//最新版本V4版本，协议使用V7，支持6面校准
	
#endif
}

//AnoOF_GetOneByte是初级数据解析函数，串口每接收到一字节光流数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数AnoOF_DataAnl
void AnoOF_GetOneByte(uint8_t data)
{
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
	else if (rxstate == 2)
	{
		rxstate = 3;
		_datatemp[2] = data;
	}
	else if (rxstate == 3 && data < OF_TEMP_LEN-5)
	{
		rxstate = 4;
		_datatemp[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		_datatemp[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
	}
	else if (rxstate == 5)
	{
		rxstate = 6;
		_datatemp[4 + _data_cnt++] = data;
	}
	else if (rxstate == 6)
	{
		rxstate = 0;
		_datatemp[4 + _data_cnt] = data;
		//		DT_data_cnt = _data_cnt+5;
		//
		AnoOF_DataAnl(_datatemp, _data_cnt + 5); //
	}
	else
	{
		rxstate = 0;
	}
}
//AnoOF_DataAnl为光流数据解析函数，可以通过本函数得到光流模块输出的各项数据
//具体数据的意义，请参照匿名光流模块使用手册，有详细的介绍

static void AnoOF_DataAnl(uint8_t *data, uint8_t len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	if (*(data + 3) != (len - 6)) //判断数据长度是否正确
		return;
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //判断sum校验
		return;
	//================================================================================

	if (*(data + 2) == 0X51) //光流信息
	{
		if (*(data + 4) == 0) //原始光流信息
		{
			ano_of.of0_sta = *(data + 5);
			ano_of.of0_dx = *(data + 6);
			ano_of.of0_dy = *(data + 7);
			ano_of.of_quality = *(data + 8);
		}
		else if (*(data + 4) == 1) //高度融合后光流信息
		{
			ano_of.of1_sta = *(data + 5);
			ano_of.of1_dx = *((s16 *)(data + 6));
			ano_of.of1_dy = *((s16 *)(data + 8));
			ano_of.of_quality = *(data + 10);
			//
			check_time_ms[1] = 0;
			ano_of.of_update_cnt++;
		}
		else if (*(data + 4) == 2) //惯导融合后光流信息
		{
			ano_of.of2_sta = *(data + 5);
			ano_of.of2_dx = *((s16 *)(data + 6));
			ano_of.of2_dy = *((s16 *)(data + 8));
			ano_of.of2_dx_fix = *((s16 *)(data + 10));
			ano_of.of2_dy_fix = *((s16 *)(data + 12));
			ano_of.intergral_x = *((s16 *)(data + 14));
			ano_of.intergral_y = *((s16 *)(data + 16));
			ano_of.of_quality = *(data + 18);
			//
		}
	}
	else if (*(data + 2) == 0X34) //高度信息
	{
		ano_of.of_alt_cm = *((u32 *)(data + 7));
		//
		check_time_ms[2] = 0;
		ano_of.alt_update_cnt++;
	}
	else if (*(data + 2) == 0X01) //惯性数据
	{
		ano_of.acc_data_x = *((s16 *)(data + 4));
		ano_of.acc_data_y = *((s16 *)(data + 6));
		ano_of.acc_data_z = *((s16 *)(data + 8));
		ano_of.gyr_data_x = *((s16 *)(data + 10));
		ano_of.gyr_data_y = *((s16 *)(data + 12));
		ano_of.gyr_data_z = *((s16 *)(data + 14));
		//shock_sta+16
	}
	else if (*(data + 2) == 0X04) //姿态信息
	{
		//四元数格式
		ano_of.quaternion[0] = (*((s16 *)(data + 4))) * 0.0001f;
		ano_of.quaternion[1] = (*((s16 *)(data + 6))) * 0.0001f;
		ano_of.quaternion[2] = (*((s16 *)(data + 8))) * 0.0001f;
		ano_of.quaternion[3] = (*((s16 *)(data + 10))) * 0.0001f;
	}
}



/********************************光流v3****************************************************/

/*
OF_STATE :
0bit: 1-高度有效；0-高度无效
1bit: 1-光流有效；0-光流无效
2bit: 1-高度融合有效；0-高度融合无效
3bit: 1-光流融合有效；0-光流融合无效
4:0
7bit: 1
*/

uint8_t		OF_STATE,OF_QUALITY;
int8_t		OF_DX,OF_DY;
int16_t		OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
uint16_t	OF_ALT,OF_ALT2;
int16_t		OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
int16_t		OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
int16_t		OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
int16_t		OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
float		OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
float		OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;

static uint8_t _datatemp[50];
static u8 _data_cnt = 0;
//static u8 anoof_data_ok;

//AnoOF_GetOneByte是初级数据解析函数，串口每接收到一字节光流数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数AnoOF_DataAnl
void AnoOF_GetOneByte_v3(uint8_t data)
{

	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0x22)	//源地址
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2)			//目的地址
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3)			//功能字
	{
		state = 4;
		_datatemp[3]=data;
	}
	else if(state==4)			//长度
	{
		state = 5;
		_datatemp[4]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		_datatemp[5+_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		_datatemp[5+_data_cnt]=data;
		check_time_ms[0] = 0;
		AnoOF_DataAnl_v3(_datatemp,_data_cnt+6);//anoof_data_ok = 1 ;//
	}
	else
		state = 0;
}


void AnoOF_DataAnl_v3(uint8_t *data_buf,uint8_t num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	
	if(*(data_buf+3)==0X51)//光流信息
	{
		if(*(data_buf+5)==0)//原始光流信息
		{
			OF_STATE 		= *(data_buf+6);
			OF_DX  		= *(data_buf+7);
			OF_DY  		= *(data_buf+8);
			ano_of.of_quality  	= *(data_buf+9);
		}
		else if(*(data_buf+5)==1)//融合后光流信息
		{
			ano_of.of1_sta 		= *(data_buf+6);
			ano_of.of1_dx		= (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			ano_of.of1_dy		= (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			
			ano_of.of2_dx_fix	= (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			ano_of.of2_dy_fix	= (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			
			ano_of.of_quality  	= *(data_buf+19);
			
			check_time_ms[1] = 0;
			ano_of.of_update_cnt++;
		}
	}
	if(*(data_buf+3)==0X52)//高度信息
	{
		if(*(data_buf+5)==0)//原始高度信息
		{
			ano_of.of_alt_cm = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			check_time_ms[2] = 0;
			ano_of.alt_update_cnt++;
		}
		else if(*(data_buf+5)==1)//融合后高度信息
		{
			OF_ALT2 = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
		}
	}
	if(*(data_buf+3)==0X53)//惯性数据
	{
		if(*(data_buf+5)==0)//原始数据
		{
			OF_GYR_X = (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			OF_GYR_Y = (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) ;
			OF_GYR_Z = (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) ;
			OF_ACC_X = (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) ;
			OF_ACC_Y = (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) ;
			OF_ACC_Z = (int16_t)(*(data_buf+16)<<8)|*(data_buf+17) ;
		}
		else if(*(data_buf+5)==1)//滤波后数据
		{
			OF_GYR_X2 = (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			OF_GYR_Y2 = (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) ;
			OF_GYR_Z2 = (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) ;
			OF_ACC_X2 = (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) ;
			OF_ACC_Y2 = (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) ;
			OF_ACC_Z2 = (int16_t)(*(data_buf+16)<<8)|*(data_buf+17) ;
		}
	}
}
