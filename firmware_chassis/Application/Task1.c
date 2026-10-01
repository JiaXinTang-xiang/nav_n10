#include "Task1.h"

extern int Run;
void Basics_One()
{
	static uint16_t Oneflag=0;
	switch (Oneflag)
	{
		case 0:
			CanMotor_CanPositionMode_Run(0x01,3,50,190000,0,0x02,3,50,190000,0);
			Delay_s(5);
			Oneflag=1;
			break;//1
		case 1:
			if( Motor_Turn(90) == 0){}
			else Oneflag=2;			
			break;
		case 2:
			CanMotor_CanPositionMode_Run(0x01,3,50,170000,0,0x02,3,50,170000,0);
			Delay_s(5);Run=0;
			Oneflag=3;
			break;//2
		case 3:
			if( Motor_Turn(-90) == 0){}
			else Oneflag=4;			
			break;
		case 4:
			CanMotor_CanPositionMode_Run(0x01,3,50,170000,0,0x02,3,50,170000,0);
			Delay_s(5);
			Oneflag=5;Run=0;
			break;//3
		case 5:
			if( Motor_Turn(90) == 0){}
			else Oneflag=6;			
			break;
		case 6:
			CanMotor_CanPositionMode_Run(0x01,3,50,170000,0,0x02,3,50,170000,0);
			Delay_s(5);
			Oneflag=7;Run=0;
			break;//4
		case 7:
			if( Motor_Turn(90) == 0){}
			else Oneflag=8;			
			break;
		case 8:
			CanMotor_CanPositionMode_Run(0x01,3,50,170000,0,0x02,3,50,170000,0);
			Delay_s(5);
			Oneflag=9;Run=0;
			break;//5
		case 9:
			if( Motor_Turn(90) == 0){}
			else Oneflag=10;			
			break;
		case 10:
			CanMotor_CanPositionMode_Run(0x01,3,50,165000,0,0x02,3,50,165000,0);
			Delay_s(5);
			Oneflag=11;Run=0;
			break;//6
		case 11:
			if( Motor_Turn(-91) == 0){}
			else Oneflag=12;			
			break;
		case 12:
			CanMotor_CanPositionMode_Run(0x01,3,50,170000,0,0x02,3,50,170000,0);
			Delay_s(5);
			Oneflag=13;Run=0;
			break;//7
		case 13:
			if( Motor_Turn(-90) == 0){}
			else Oneflag=14;			
			break;
		case 14:
			CanMotor_CanPositionMode_Run(0x01,3,50,165000,0,0x02,3,50,165000,0);
			Delay_s(5);
			Oneflag=15;Run=0;
			break;//8
		
	  
		
		
				
				
				
				
				
	}
	
}




/*				Make by 轩
 *
 * 函数名: CanMotor_CanSpeedMode_Run
 * 描述:   通过CAN总线控制两个步进电机以位置模式运行
 * 参数:   address  - 电机1的自定义协议地址0x01
 *         Accel    - 电机1的加速度( 0-400 )
 *         onespeed - 电机1的目标速度( 范围: 0-2000 )
 *         position1- 电机1的目标位置( 单位: 步长，96000轮子一圈 )
 *         adr1     - 电机1的方向控制位( 0表示正转，1表示反转 )
*/

