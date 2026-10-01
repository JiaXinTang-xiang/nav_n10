#include "IMU_TurnControl.h"

pid_type_def imu_pid;                     // PID结构体
float imu_params[3] = {1.0f, 0.0f, 0.0f}; // PID参数
extern float ypr[3];

static uint8_t IMU_init_done = 0; // 标记PID是否已初始化

uint8_t Run    = 0;     /* 1=正在转弯  0=空闲，可接收新指令 */
float   target = 0.0f;  /* 目标 yaw 角度 */

/**
 * @brief  设目标角度并启动转弯（只设目标，不跑 PID）
 * @param  Turn_angle  旋转度数(范围: -180°~ 180°)
 */
void Motor_Turn(float Turn_angle)
{
    target = ypr[0] - Turn_angle;
    Run    = 1;
}

/**
 * @brief  执行一轮 PID 调控（由定时器按频率调用）
 * @note   读当前 yaw → 最短路径误差 → PID_calc → Load
 */
void IMU_Turn_Run(void)
{
    float Yaw_now = ypr[0];

    // 最短路径误差
    float error = target - Yaw_now;
    if (error > 180.0f)       error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    float real_target = Yaw_now + error;

    PID_calc(&imu_pid, Yaw_now, real_target);

    // 误差 < 0.5° → 停
    if (fabsf(error) <= 0.5f) {
        imu_pid.out = 0;
        Run = 0;              // 转弯完成，回到空闲
    }

    Load(-imu_pid.out, imu_pid.out);
}

/**
 * @brief  PID初始化函数
 * @param  无
 * @retval 无
 */
void IMU_Ctrl_Init()
{
    // 仅初始化一次PID (避免每次循环重置积分项)
    if (!IMU_init_done)
    {
        // 角度参数：1.PID结构体 2.位置式 3.PID参数数组 4.输出限幅 5.积分限幅
        PID_init(&imu_pid, PID_POSITION, imu_params, 20, 0);
        IMU_init_done = 1;
    }
}

/* ================================================================
 *                     main.c 调用示例 (供参考)
 * ================================================================
 *
 *   本模块通过 IMU 的 yaw 角实现闭环转向控制，
 *   由 TB6612 电机驱动两轮差速完成转弯。
 *   调用方只需设目标角度，其余由定时器自动完成。
 *
 *   // -------------------- 初始化 (main.c) --------------------
 *   // 在 main() 中调用一次，初始化位置式 PID
 *   IMU_Ctrl_Init();
 *
 *   // -------------------- 启动转弯 (任意位置) --------------------
 *   // 相对当前朝向旋转指定度数 (范围 -180° ~ +180°)
 *   // +90° = 顺时针转 90°, -45° = 逆时针转 45°
 *   if (!Run)  Motor_Turn(90);    // 右转 90°
 *   if (!Run)  Motor_Turn(-45);   // 左转 45°
 *   if (!Run)  Motor_Turn(180);   // 掉头
 *
 *   // -------------------- 定时器回调 (Timer_task.c) --------------------
 *   // 放在定时器中断/Task里，例如 100Hz 周期性调用
 *   // 内部自动: 读yaw → 算最短路径误差 → PID计算 → Load(左轮, 右轮)
 *   //           误差 < 0.5° 时自动停车并置 Run=0
 *   if (Run)  IMU_Turn_Run();
 *
 *   // -------------------- 查询是否在转弯 --------------------
 *   // Run=1 表示正在转弯中，Run=0 表示空闲可接收新指令
 *   if (Run) {
 *       printf("转弯中... 目标yaw=%.1f\r\n", target);
 *   }
 *
 *   // -------------------- PID参数整定 --------------------
 *   // 修改 imu_params[3] 即可调整响应 (在文件顶部或 extern 处)
 *   // imu_params[0] = Kp   (比例, 默认1.0 — 越大越猛)
 *   // imu_params[1] = Ki   (积分, 默认0.0)
 *   // imu_params[2] = Kd   (微分, 默认0.0)
 *   // 输出限幅: PID_init(..., 20, 0) → ±20
 *
 *   ============================================================
 *   工作流程:
 *      Motor_Turn(角度)          // 1. 设目标并启动
 *        → Run = 1, target = 当前yaw + 角度
 *      IMU_Turn_Run()            // 2. 定时器按周期执行
 *        → yaw → 最短路径误差 → PID → Load(左,右)
 *        → 误差<0.5° → Run=0     // 3. 到位自动停
 *
 *   依赖:
 *     ypr[0]         — IMU 模块提供的当前 yaw 角 (extern)
 *     PID_calc()     — pid.c 提供的 PID 计算函数
 *     Load(L, R)     — TB6612.c 提供的双轮差速驱动
 *   ============================================================
 */
