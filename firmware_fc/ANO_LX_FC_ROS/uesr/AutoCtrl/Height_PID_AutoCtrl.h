#ifndef _HEIGHT_PID_AUTOCTRL_H_
#define	_HEIGHT_PID_AUTOCTRL_H_

//м╥нд╪Ч
#include "stm32f4xx.h"
#include "Ano_Pid.h"

typedef struct
{
	float exp;
	float fb;
	float out;
}_halt_ctrl_st;

void Height_PID_Init(void);
void Fc_Height_PID(float dt,float exp);
extern int height;

#endif



