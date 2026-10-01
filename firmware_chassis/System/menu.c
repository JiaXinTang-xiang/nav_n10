#include "stm32f10x.h"       
#include "menu.h"
#include "OLED.h"
#include "Key.h"
#include "Delay.h"

typedef enum {
    Menu_Main = 0,
    Menu_Set_Length,
    Menu_Set_SetThestep,
   
} MenuState;

typedef enum {
	gostrenth= 0,
    turnleft, 
    turnrigt,
}type;


		
MenuState menu_state = 0;
type acctype=0;
u16 Route;
u16 chosestep;
u32 golength[20];
u8 k=0;
u8 keynum;

// 显示主菜单
void OLED_ShowMainMenu(void)
{
    OLED_Clear();
	
	OLED_ShowString(1, 3, "1.Route Leng");
	OLED_ShowString(2, 3, "2.choose TheStep");
    OLED_ShowString(3, 3, "3.set TheStep");
   
}

// 显示设置任务长度界面
void OLED_ShowSetLength(void)
{
    OLED_Clear();
    OLED_ShowString(1, 3, "Set R:");
    OLED_ShowNum(1, 12,Route, 2);
    OLED_ShowString(3, 3, "Press l to inc");
    OLED_ShowString(4, 3, "Press r 0k");
}
//选择设置的步
void OLED_ShowSetThestepchose(void)
{
    OLED_Clear();
    OLED_ShowString(1, 3, "chosestep:");
	OLED_ShowString(3, 3, "Press l to inc");
    OLED_ShowString(4, 3, "Press r 0k");
}
//选择该步类型
void OLED_ShowSetThestep(void)
{
    OLED_Clear();
    OLED_ShowString(1, 3, "TurnLeft");
	OLED_ShowString(2, 3, "TurnRight");
	OLED_ShowString(3, 3, "Go straight");
	OLED_ShowString(4, 3, "exit");

}
//选择直线时需要选择步长，默认150000，每次按下按钮加5000
void OLED_ShowSetgo(void)
{
    OLED_Clear();
    OLED_ShowString(1, 3, "SetThegolength");
	OLED_ShowNum(2, 3,golength[chosestep], 2);
    OLED_ShowString(4, 3, "0k");
}

int menu_main(void)//主菜单设置
{  
     int cflag=1;
	 OLED_ShowMainMenu();
     while(1)
     {  keynum=Key_GetNum();
        OLED_ShowString(cflag,1,"->");
	  	if(keynum==1)
		{
			cflag++;
			for(int i=1;i<=4;i++)
			{
				 OLED_ShowString(i,1,"  ") ;
			}
			if(cflag>=4)cflag=1;		
		}
        if(keynum==2)
	   {
            switch(cflag)
            {  
	             case 1:menu_SetLength();break;
	             case 2:menu_SetThestepchose();break;
                 case 3:menu_SetTheStep();break;
                 case 4:break;
            }
      }
  }
}
int menu_SetLength()
{     int cflag=1;
      OLED_ShowSetLength();

 while(1)
     { 
		 keynum=Key_GetNum();
         OLED_ShowString(cflag,1,"->");
		 Delay_ms(50);
	  	if(keynum==1)
		{   Route++;
			OLED_ShowString(1,12,"  ");
			OLED_ShowNum(1, 12,Route, 2);	
			if(Route>=20)Route=0;
		}
        if(keynum==2)
	   {    
		  
		   keynum=0;
		   OLED_ShowMainMenu();
		   return 0;
       }
     }



}

int menu_SetThestepchose(void)
{
    

      int cflag=1;
       OLED_ShowSetThestepchose();
       OLED_ShowNum(1, 13,1,2);
 while(1)
     {  
		 keynum=Key_GetNum();
         OLED_ShowString(cflag,1,"->");
		 
	  	if(keynum==1)
		{   k++;
			OLED_ShowString(1,13,"  ");
			OLED_ShowNum(1, 13,k,2);	
			if(k>=Route)k=0;
			
		}
        if(keynum==2)
	   {    

		   keynum=0;
		   OLED_ShowMainMenu();
           return 0;
 
       }
     }

}


int menu_SetTheStep(void)//主菜单设置
{  
     int cflag=1;
	 OLED_ShowSetThestep();
     while(1)
     {  keynum=Key_GetNum();
        OLED_ShowString(cflag,1,"->");
	  	if(keynum==1)
		{
			cflag++;
			for(int i=1;i<=4;i++)
			{
				 OLED_ShowString(i,1,"  ") ;
			}
			if(cflag>=5)cflag=1;		
		}
        if(keynum==2)
	   {
            switch(cflag)
            {  
	             case 1:break;
	             case 2:break;
                 case 3:break;
                 case 4:keynum=0;OLED_ShowMainMenu();return 0; break;
		                             
            }
      }
  }
}



