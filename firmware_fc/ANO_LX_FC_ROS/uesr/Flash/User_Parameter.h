#ifndef _USER_PARAMETER_H_
#define _USER_PARAMETER_H_
#include "stm32f4xx.h"

__packed struct Parameter_s
{
	u16 frist_init;	//飞控第一次初始化，需要做一些特殊工作，比如清空flash
	
};


union Parameter
{
	//这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数
	struct Parameter_s set;
	u8 byte[4096];
};
extern union Parameter Ano_Parame;

typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
}_parameter_state_st ;
extern _parameter_state_st para_sta;

void Ano_Parame_Read(void);
void Ano_Parame_Write_task(u16 dT_ms);
void Parameter_DATA_Rest(void);
static void Ano_Parame_Write(void);

void Para_Data_Init(void);
void All_PID_Init(void);
void data_save(void);


#endif










