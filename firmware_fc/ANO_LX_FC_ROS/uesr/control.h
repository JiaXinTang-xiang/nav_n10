#ifndef __CONTROL_H
#define __CONTROL_H

extern int control_flag;
extern int time,time_cnt_ms;
extern unsigned int  mission_step,mission_step1,mission_step2,mission_step3;

void all_data_init(void);
void User_Task_Delay(int ms);
void User_Task_Delay1(int ms);
void User_Task_Delay2(int ms);
void User_Task_Delay3(int ms);
void User_Task_Delay4(int ms);
int square_wave(int amplitude);
int triangualr_wave(int amplitude);

#endif
