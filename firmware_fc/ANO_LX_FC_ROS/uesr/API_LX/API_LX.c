#include "API_LX.h"
#include "Ano_Math.h"
#include "Drv_AnoOf.h"

/***********************************************************
*@函  数	:API_Take_Off
*@功  能	:凌霄飞控起飞函数
*@参  数	:时间累加
*@返回值	:返回1
*@备  注	:--
*@时  间	:2023-04-05
***********************************************************/
u8 API_Take_Off(u16 dt,u16 height_cm)
{
	static u8 return_value; //返回值
	static u8 pattern = 0;
	static u16 dt_ms;
	switch(pattern)
	{
		case 0:
		{
			dt_ms = 0;
			pattern += 1;
			
		}break;
		case 1:
		{
			if(dt_ms<5000){dt_ms+=dt;} //使用按键时保证人员安全
			else{dt_ms=0;pattern++;}
			
		}break;
		case 2:
		{
			LxStringSend(2,(char*)"定点模式");
			pattern += LX_Change_Mode(2);//定点模式
			
		}break;
		case 3:
		{
			pattern+=FC_Unlock();//解锁
		}break;
		case 4:
		{
			if(dt_ms<2000){dt_ms+=20;} //延时
			else{dt_ms=0;pattern++;}
			
		}break;
		case 5:
		{
			LxStringSend(2,(char*)"起飞高度150cm");
			pattern+=OneKey_Takeoff(height_cm);//起飞高度150cm			
		}break;
		case 6:
		{
			if(dt_ms < 3500) //延时
			{
				dt_ms += dt;
			}
			else
			{
				return_value = 1; //起飞任务完成，返回值1
				LxStringSend(2,(char*)"taking_off_ok");
				dt_ms = 0;
				pattern ++;
			}
		}break;
		default:
			break;
	}
	return return_value;	
}

/***********************************************************
*@函  数	:API_landing
*@功  能	:凌霄飞控下降函数
*@参  数	:时间累加
*@返回值	:返回1
*@备  注	:--
*@时  间	:2023-04-05
***********************************************************/

u8 API_landing(u16 dt)
{
	static u8 return_value; //返回值
	static u8 pattern = 0;
	static u16 dt_ms;
	
	switch(pattern)
	{
		case 0:
		{
			dt_ms = 0;
			pattern += 1;
		}break;
		case 1:
		{
			if(dt_ms < 2000)
			{
				dt_ms += dt;	
			}
			else
			{
				dt_ms = 0;
				pattern += 1;
			}				
		}break;
		case 2:
		{
			//执行一键降落
			OneKey_Land();	
		}break;
		case 3:
		{
			if(dt_ms < 3000)
			{
				dt_ms += dt;	
			}
			else
			{
				dt_ms = 0;
				pattern += 1;
			}				
			
		}break;
		case 4:
		{
			LxStringSend(2,(char*)"上锁");
			FC_Lock();
			return_value = 1;
		}break;
		default:
			break;
	}
	return return_value;
}

/***********************************************************
*@fuction	:Reset_vel
*@brief		:速度清零函数
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-07-04
***********************************************************/
void Reset_vel(void)
{
	/*优化后代码*/
	fc.acc_x = 0;
	fc.acc_y = 0;
	fc.acc_z = 0;
	fc.yaw = 0;
}

//用户延时函数
uint8_t API_User_Delay(uint32_t dt)
{
	static uint32_t dt_ms;
	if(dt_ms < dt){dt_ms += 20;}
	else {dt_ms = 0;}
	return 1;
}

/*******************************************************************************************************************/
/*******************************************************************************************************************/
/*******************************************************************************************************************/

// 定义路径点数组
Vector2 waypoints[] = 
{
	{0.0f,     0.0f},         /*无人机的初始位置，不能进行修改*/
	{100.0f,   100.0f},
	{50.0f,    100.0f},
	{5.0f,     -5.0f},
	{0.0f,     0.0f}
};

Drone drone;
// 初始化无人机
void Drone_Init(Drone* drone, Vector2* waypoints) 
{
		//计算坐标数量
		int waypoint_count = sizeof(waypoints) / sizeof(waypoints[0]);
	
    // 设置初始位置为第一个路径点
    drone->position = waypoints[0];
    drone->velocity.x = 0.0f;
    drone->velocity.y = 0.0f;
    
    // 设置路径点
    drone->waypoints = waypoints;
    drone->waypoint_count = waypoint_count;
    drone->current_waypoint = 0;
    drone->is_hovering = 0;
    drone->hover_complete = 0;
	
    //printf("无人机初始化完成，共有 %d 个路径点\n", waypoint_count);
}

// 计算两点之间的距离
float distance(Vector2 a, Vector2 b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}



// 更新无人机状态
void Drone_Update(Drone* drone, float dt) {
    // 检查是否完成所有路径点
    if (drone->current_waypoint >= drone->waypoint_count) {
        //printf("所有路径点已完成！\n");
        return;
    }
    
    Vector2 target = drone->waypoints[drone->current_waypoint];
    
    // 如果正在悬停
    if (drone->is_hovering) {
        //clock_t current_time = clock();
        //double hover_time = (double)(current_time - drone->hover_start) / CLOCKS_PER_SEC;
        
        // 检查悬停时间是否达到5秒
//        if (hover_time >= 5.0) {
//            drone->is_hovering = 0;
//            drone->hover_complete = 1;
//            printf("完成在路径点 (%.2f, %.2f) 的悬停，前往下一个路径点\n",  target.x, target.y);
//            drone->current_waypoint++;
//        } else {
//            // 悬停中，保持位置
//            printf("正在路径点 (%.2f, %.2f) 悬停，剩余时间: %.2f秒\n", target.x, target.y, 5.0 - hover_time);
//            return;
//        }
    }
    
    // 如果完成了悬停但还有下一个路径点
    if (drone->hover_complete && drone->current_waypoint < drone->waypoint_count) 
			{
        drone->hover_complete = 0;
        target = drone->waypoints[drone->current_waypoint];
        //printf("前往路径点 %d: (%.2f, %.2f)\n", drone->current_waypoint + 1, target.x, target.y);
               
    }
    
    // 计算当前位置与目标点的距离
    float dist = distance(drone->position, target);
    
    // 如果到达目标点附近，开始悬停
    if (dist < 0.1f) {
        //printf("到达路径点 %d: (%.2f, %.2f)，开始悬停5秒\n", drone->current_waypoint + 1, target.x, target.y);    
        drone->is_hovering = 1;
        //drone->hover_start = clock();
        return;
    }
    
    // 使用PID控制器计算加速度
    //float accel_x = PID_Update(&drone->pid_x, target.x, drone->position.x, dt);
    //float accel_y = PID_Update(&drone->pid_y, target.y, drone->position.y, dt);
    
    // 更新速度
    //drone->velocity.x += accel_x * dt;
    //drone->velocity.y += accel_y * dt;
    
    // 更新位置
    drone->position.x += drone->velocity.x * dt;
    drone->position.y += drone->velocity.y * dt;
    
    // 输出当前状态
    //printf("位置: (%.2f, %.2f), 目标: (%.2f, %.2f), 距离: %.2f\n", drone->position.x, drone->position.y, target.x, target.y, dist);
          
          
}














/*
* 初始化
*
*
*/
void API_Drone_Init(void)
{
	Drone_Init(&drone,waypoints);
}





/*******************************************************************************************************************/
/*******************************************************************************************************************/
/*******************************************************************************************************************/


