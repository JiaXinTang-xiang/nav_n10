#ifndef __ANO_DT_LX_H
#define __ANO_DT_LX_H
//==引用
#include "SysConfig.h"

//==定义/声明
#define FUN_NUM_LEN 256

typedef struct
{
	u8 D_Addr;		 //目标地址
	u8 WTS;			 //wait to send等待发送标记
	u16 fre_ms;		 //发送周期
	u16 time_cnt_ms; //计时变量
} _dt_frame_st;

//cmd
typedef struct
{
	u8 CID;
	u8 CMD[10];
} _cmd_st;

//check
typedef struct
{
	u8 ID;
	u8 SC;
	u8 AC;
} _ck_st;

//param
typedef struct
{
	u16 par_id;
	s32 par_val;
} _par_st;

typedef struct
{
	_dt_frame_st fun[FUN_NUM_LEN];
	//
	u8 wait_ck;
	//
	_cmd_st cmd_send;
	_ck_st ck_send;
	_ck_st ck_back;
	_par_st par_data;
} _dt_st;

//==数据声明
extern _dt_st dt;
//==函数声明

//public
//
void AnoDTIMUInit(void);
void AnoDTIMURunTask(float dT_s);
void AnoDTIMURecvOneByte(u8 data);
//
void CMD_Send(u8 dest_addr, _cmd_st *cmd);
void CK_Back(u8 dest_addr, _ck_st *ck);
void PAR_Back(u8 dest_addr, _par_st *par);
//用户常用调试函数
void LxF1Send(u8 u8val, u16 u16val, s16 s16val, s32 s32val);
void LxF1SendPosition(s32 val1,s32 val2,s32 val3);
void LxF1SendSpeed(s16 val1,s16 val2,s16 val3,s16 yaw);
void LxStringSend(u8 string_color,char *str);
void LxF1Send_9(s16 val1,s16 val2,s16 val3,s16 d1,s16 d2,s16 d3,s16 d4,s16 d5,s16 d6,u32 d7);
#endif
