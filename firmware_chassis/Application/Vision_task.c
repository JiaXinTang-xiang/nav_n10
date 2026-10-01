#include "Vision_task.h"
#include "Serial.h"
#include "bsp.h"

pid_type_def imu_pid;                     // PID结构体
float imu_params[3] = {1.0f, 0.0f, 0.0f}; // PID参数
float Yaw_now = 0;                        // 当前角度

pid_type_def Jetson_pid;                     // PID结构体
float Jetson_params[3] = {0.5f, 0.0f, 0.1f}; // PID参数
float Target_x = 0;                          // 目标标
float Motor_out;                             // 电机输出
float target_Motor = 40.0f;                  // 电机输出

static uint8_t pid_init_done = 0; // 标记PID是否已初始化


void Vision_task()
{
    Motor_out = PID_calc(&Jetson_pid, center_x, Target_x);                             // 参数：1.PID结构体 2.视觉中心线的当前值 3.视觉目标值
    CanMotor_CanSpeedMode_Run2(Motor_out,  Motor_out, 5); // 参数：1.左轮速度  2.右轮速度（0-2000） 3.加速度（0-400）
}



/**
 * @brief  步进电机转弯函数
 * @param  旋转度数( 范围: -180°~ 180° )
 * @retval 无
 */
extern int Run;
int Motor_Turn(float Turn_angle)
{
	static float error;
	static float target;
	static float finish;
	
    Yaw_now = Xreadangle.Yaw;
	
	if(!Run){
		target = Yaw_now - Turn_angle;	
		Run=1;
		finish=0;
	}
	
    // 计算目标角度与当前角度的最短路径
    error = target - Yaw_now;

    if (error > 180.0f)   error -= 360.0f;
    else if (error < -180.0f)  error += 360.0f;

    target = Yaw_now + error;

    PID_calc(&imu_pid, Yaw_now, target);

    // 误差小于1°停止
    if(abs((int)error) <= 0.5 ){
        imu_pid.out = 0;
		finish=1;
    }

    CanMotor_CanSpeedMode_Run2(-imu_pid.out, +imu_pid.out, 4);
	return finish;
}

/**
 * @brief  PID初始化函数
 * @param  无
 * @retval 无
 */
void PID_Ctrl_Init()
{
    // 仅初始化一次PID (避免每次循环重置积分项)
    if (!pid_init_done)
    {
        // 视觉环参数：1.PID结构体 2.位置式 3.PID参数数组 4.输出限幅 5.积分限幅
        PID_init(&Jetson_pid, PID_POSITION, Jetson_params, 30, 0);

        // 角度参数：1.PID结构体 2.位置式 3.PID参数数组 4.输出限幅 5.积分限幅
        PID_init(&imu_pid, PID_POSITION, imu_params, 50, 0);
		
        pid_init_done = 1;
    }
}

