#include "User_Task1.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "AnoDTRasp.h"
#include "API_LX.h"
#include "control.h"
#include "mode.h"
#include "Pos_PID_AutoCtrl.h"
#include "Height_PID_AutoCtrl.h"
#include "Drv_AnoOf.h"
#include "Ano_Math.h"
#include "Drv_Uart.h"
#include "my_uart.h"

///////基础部分：投掷一个物块，起飞悬停3s，按航线走，不超过30cm，降落离中心不超过20cm///////
//////时间不超过180s，物块50cm内///////
u8 Send_32[3] = {0x01,0x02,0x03};

void UserTask_OneKeyCmd1(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
//    static u8  one_key_land_f = 1;
	  static u8 one_key_mission_f1 = 0;
//    static u8 one_key_takeoff_f = 1
    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
//        //判断第6通道拨杆位置 1300<CH_6<1700
//        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
//        {
//            //还没有执行
//            if (one_key_takeoff_f == 0)
//            {
//                //标记已经执行
//                one_key_takeoff_f =
//                    //执行一键起飞
//                    OneKey_Takeoff(100); //参数单位：厘米； 0：默认上位机设置的高度。
//            }
//        }
//				
//        else
//        {
//            //复位标记，以便再次执行
//            one_key_takeoff_f = 0;
//        }
        //
//        //判断第6通道拨杆位置 800<CH_6<1200
//        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
//        {
//            //还没有执行
//            if (one_key_land_f == 0)
//            {
//                //标记已经执行
//                one_key_land_f =
//                    //执行一键降落
//                    OneKey_Land();
//            }
//        }
//        else
//        {
//            //复位标记，以便再次执行
//            one_key_land_f = 0;
//        }
        //判断第7通道拨杆位置 1700<CH_7<2000
        if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1700 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2200)
        {
            //还没有执行
            if (one_key_mission_f1 == 0)
            {
                //标记已经执行
                one_key_mission_f1 = 1;
                //开始流程
                mission_step1 = 1;
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_mission_f1 = 0;
        }
        //
        if (one_key_mission_f1 == 1)
        {		
					switch(mission_step1)
					{
						case 0:
						{
							time = 0;
						}break;
						case 1:
						{
							time = 0;
							mission_step1 ++;
						}break;
						case 2:
						{
							User_Task_Delay1(2000);
						}break;
						case 3:
						{
							mission_step1 += LX_Change_Mode(2);
						}break;
						case 4:
						{ 
							all_data_init();
							mode_select(0); //激光雷达定位	
							mission_step1 +=FC_Unlock();//解锁						
						}break;
						case 5:
						{
								User_Task_Delay1(2000);//延时2s		
						}break;
						case 6:
						{							
							 mode_select(4); //激光雷达定位							 
//							 fc_loc_exp.LocExpX=0;
//							 fc_loc_exp.LocExpY=0;
							 height=85;	//起飞到0.85m
							 User_Task_Delay1(3000);	
						}break;  									 
					 case 7:
					 { 
							 fc_loc_exp.LocExpX=0;
							 fc_loc_exp.LocExpY=0;
							 User_Task_Delay1(4000);	//悬停
					 }break;						 
					 case 8:	
					 {						 
								dw_select(1);		//第一个点位：x=160，y=0	，向前飞1.6m					
								mission_step1++;					 
					 }break;						
					 case 9:	
					 {
							 if( fc_loc_exp.LocExpX+rosData.loc[0]>-5&&fc_loc_exp.LocExpX+rosData.loc[0]<5 &&
									 fc_loc_exp.LocExpY+rosData.loc[1]>-5&&fc_loc_exp.LocExpY+rosData.loc[1]<5)
							 { 							   						 
									mission_step1++;//判断是否到达，是就step++
							 }
					 }break;
					 case 10:	
					 {							
								 fc_loc_exp.LocExpX=160;
							   fc_loc_exp.LocExpY=0;
					 	  	 User_Task_Delay1(2000);
					 }break;
					 case 11:	
					 {
									dw_select(2);		//第二个点位：x=160，y=-73	，向右飞0.8m					
									mission_step1++;
					 }break;
						case 12:
						{
								if( fc_loc_exp.LocExpX+rosData.loc[0]>-5&&fc_loc_exp.LocExpX+rosData.loc[0]<5 &&
									 fc_loc_exp.LocExpY+rosData.loc[1]>-5&&fc_loc_exp.LocExpY+rosData.loc[1]<5)
								{ 
										mission_step1++;//判断是否到达，是就step++
								}
						}break;
						case 13 :
						{
							  Send_Data_To_32(&Send_32[0],1);//声光提示
							  fc_loc_exp.LocExpX=158;
						 	  fc_loc_exp.LocExpY=-74;
								User_Task_Delay1(3000);	
					  }break;						 
						case 14:
						{						
								fc_loc_exp.LocExpX=158;
							  fc_loc_exp.LocExpY=-74;								
								User_Task_Delay1(1000);                   
						}break;   
						case 15:
						{
							  Send_Data_To_32(&Send_32[2],1);//左边投掷物块
							  mission_step1++;						                 
						}break; 
						case 16:
						{
								fc_loc_exp.LocExpX=160;//微调位置
								fc_loc_exp.LocExpY=-74;
							  User_Task_Delay(2000);                    
						}break;  
						case 17:
						{
							  dw_select(3);		//第三个点位：x=160，y=-160,向右飞0.8m				
                mission_step1++;//判断是否到达，是就step++
						}break;
						case 18:
						{
								if( fc_loc_exp.LocExpX+rosData.loc[0]>-5&&fc_loc_exp.LocExpX+rosData.loc[0]<5 &&
									 fc_loc_exp.LocExpY+rosData.loc[1]>-5&&fc_loc_exp.LocExpY+rosData.loc[1]<5)
								{ 
										mission_step1++;//判断是否到达，是就step++
								}
						}break;
						case 19:
						{
								fc_loc_exp.LocExpX=160;
							  fc_loc_exp.LocExpY=-160;
								User_Task_Delay1(3000);
						}
						case 20:
						{
								dw_select(4);		//降落区，向后飞1.6m，第四个点位：x=0，y=-160						
								mission_step1++;                  
						}break; 
						case 21:
						{
								if( fc_loc_exp.LocExpX+rosData.loc[0]>-5&&fc_loc_exp.LocExpX+rosData.loc[0]<5 &&
									 fc_loc_exp.LocExpY+rosData.loc[1]>-5&&fc_loc_exp.LocExpY+rosData.loc[1]<5)
								{ 
										mission_step1++;//判断是否到达
								}
						}break;
						case 22:
						{
							  fc_loc_exp.LocExpX=0;
								fc_loc_exp.LocExpY=-160;
							  User_Task_Delay1(3000);//到达后延时3s降落			
						}break;		        	
						case 23:
						{
							  mission_step1=998;
						}break;		
						
						case 998:
						{						
								fc_loc_exp.LocExpX=0;     //降落
								fc_loc_exp.LocExpY=-160;
							  height=10;	
								if(height<20) 
								{										
									mission_step1=999;
								}
						}break;
						case 999:
						{ 
							    fc_loc_exp.LocExpX=0;
								  fc_loc_exp.LocExpY=-160;
									OneKey_Land();//执行一键降落	
									AnoPTv7Send_ShutDown();/*关机指令*/
						}break;
					}
				}
						else
						{
								mission_step1 = 0;
						}
    }
    ////////////////////////////////////////////////////////////////////////
}
