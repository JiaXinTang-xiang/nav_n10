#ifndef __MODE_H
#define __MODE_H

#include "stdint.h"
#include "control.h"
#include "stm32f4xx.h"                  // Device header

extern int my_mode;//·ÉÐÐÄ£Ê½
extern int dis_target_ref,dis_group[2][20];


void mode_select(int mode);
void dw_select(int dw);

#endif
