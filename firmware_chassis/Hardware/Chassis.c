#include "stm32f10x.h"
#include "Chassis.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include <math.h>

/* 速度PID */
static pid_type_def pid_left;
static pid_type_def pid_right;
static float pid_params[3];

/* 目标速度 */
static float target_speed_l = 0;  // mm/s
static float target_speed_r = 0;

/* 里程计累计 */
static float odom_x     = 0;   // mm
static float odom_y     = 0;   // mm
static float odom_theta = 0;   // mrad

/* 编码器总脉冲 (里程计用) */
static int32_t total_pulse_l = 0;
static int32_t total_pulse_r = 0;

/**
 * @brief  底盘初始化
 */
void Chassis_Init(void)
{
    /* 电机 */
    Motor_Z_Init();
    Motor_Y_Init();

    /* 编码器 */
    Encoder_Z_Init();
    Encoder_Y_Init();

    /* 速度PID — 左右轮共用参数 */
    pid_params[0] = SPEED_KP;
    pid_params[1] = SPEED_KI;
    pid_params[2] = SPEED_KD;
    PID_init(&pid_left,  PID_POSITION, pid_params, SPEED_MAX_OUT, SPEED_MAX_IOUT);
    PID_init(&pid_right, PID_POSITION, pid_params, SPEED_MAX_OUT, SPEED_MAX_IOUT);

    /* 里程计归零 */
    odom_x     = 0;
    odom_y     = 0;
    odom_theta = 0;
}

/**
 * @brief  设置底盘目标速度 (运动学逆解)
 * @param  v_linear:  线速度 (mm/s), 前进为正
 * @param  v_angular: 角速度 (mrad/s), 左转为正
 */
void Chassis_SetVelocity(float v_linear, float v_angular)
{
    float half_track = WHEEL_TRACK_MM / 2.0f;
    float omega      = v_angular / 1000.0f;  // mrad/s → rad/s

    target_speed_l = v_linear - omega * half_track;
    target_speed_r = v_linear + omega * half_track;
}

/**
 * @brief  急停
 */
void Chassis_Stop(void)
{
    target_speed_l = 0;
    target_speed_r = 0;
    Motor_Z_SetSpeed(0);
    Motor_Y_SetSpeed(0);
    PID_clear(&pid_left);
    PID_clear(&pid_right);
}

/**
 * @brief  底盘周期更新 (每10ms调用一次)
 *         编码器读取 → 速度计算 → PID闭环 → PWM输出 → 里程计累加
 */
void Chassis_Update(void)
{
    /* 1. 读取编码器增量 (读取后计数器自动清零) */
    int16_t delta_l = Encoder_Z_Get();
    int16_t delta_r = Encoder_Y_Get();

    /* 2. 累加总脉冲 */
    total_pulse_l += delta_l;
    total_pulse_r += delta_r;

    /* 3. 速度换算: delta(10ms脉冲) * 100 → 每秒脉冲 → * mm_per_count → mm/s */
    float actual_speed_l = (float)delta_l * 100.0f * MM_PER_COUNT;
    float actual_speed_r = (float)delta_r * 100.0f * MM_PER_COUNT;

    /* 4. 速度PID闭环 */
    float pwm_l = PID_calc(&pid_left,  actual_speed_l, target_speed_l);
    float pwm_r = PID_calc(&pid_right, actual_speed_r, target_speed_r);

    /* 5. 输出到电机 (右电机镜像安装, 前进需负PWM) */
    Motor_Z_SetSpeed((int16_t)pwm_l);
    Motor_Y_SetSpeed(-(int16_t)pwm_r);

    /* 6. 里程计更新 */
    float ds_l = (float)delta_l * MM_PER_COUNT;   // 左轮位移 (mm)
    float ds_r = (float)delta_r * MM_PER_COUNT;   // 右轮位移 (mm)

    float ds     = (ds_l + ds_r) / 2.0f;
    float dtheta = (ds_r - ds_l) / WHEEL_TRACK_MM;  // rad

    /* 中点法累加位姿 */
    float theta_mid = odom_theta / 1000.0f + dtheta / 2.0f;
    odom_x     += ds * cosf(theta_mid);
    odom_y     += ds * sinf(theta_mid);
    odom_theta += dtheta * 1000.0f;    // 保持 mrad 单位
}

/**
 * @brief  获取里程计
 */
void Chassis_GetOdometry(float *x, float *y, float *theta)
{
    *x     = odom_x;
    *y     = odom_y;
    *theta = odom_theta;
}
