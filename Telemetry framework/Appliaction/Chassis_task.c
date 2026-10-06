/**
 * @file    Chassis_task.c
 * @brief   底盘任务实现
 *
 * 调用链:
 *   Task_200Hz (5ms)  ->  Chassis_Update()
 *                          ├── Encoder_Update()      读增量、算轮速
 *                          ├── PID_calc() x2         左右轮速度环
 *                          └── Load()                写 PWM
 *
 * 符号约定:
 *   Load(负, 负) = 前进 (硬件实测), 所以对外统一用"正 = 前进",
 *   翻符号只发生在 Chassis_OutputRaw() 这一处。
 */

#include "Chassis_task.h"
#include "TB6612.h"
#include "pid.h"
#include "encoder.h"
#include "oled.h"
#include "IMU_45686.h"

/* nowtime: TIM6 时基计数 (单位 100us), 定义在 Timer_task.c, 声明在 tim.h */
extern volatile uint32_t nowtime;

/* ======================== 可调参数 ======================== */

/* 最大轮速限制 (m/s), 保护电机与机械 */
#define CHASSIS_MAX_WHEEL_MPS   0.60f

/* 速度环 PID —— 直接用工程现成的 pid.c (PID_init / PID_calc / pid_type_def)
 *
 * 输入: 反馈与目标都是 m/s (PID 内部按 m/s 算误差)
 * 输出: PWM ±100
 *
 * 实测标定: PWM 20 -> 261 mm/s, 所以 1 PWM ≈ 13 mm/s。
 *
 * 速度控制使用每轮独立前馈 + PD，不加入 Ki。
 * 两边相同 PWM 不保证相同轮速，因此分别用编码器反馈闭环。
 */
static const fp32 s_pid_left_param[3]  = { 70.0f, 0.0f, 5.0f };
static const fp32 s_pid_right_param[3] = { 70.0f, 0.0f, 5.0f };
#define CHASSIS_PID_MAX_OUT    100.0f
#define CHASSIS_PID_MAX_IOUT   0.0f

/* 实测 PWM=20 约 0.261m/s，作为起始前馈。左右轮可独立标定。 */
#define CHASSIS_LEFT_FF_PWM_PER_MPS   95.0f
#define CHASSIS_RIGHT_FF_PWM_PER_MPS  70.0f
#define CHASSIS_FF_STATIC_PWM          2.0f

/* 直行时的陀螺仪角速度外环。
   这里的 s_gyro_corr_w 是“需要施加的反向角速度”，所以测到车体向左转
   时，必须让左轮略快、右轮略慢。若 IMU 的 Z 轴正方向与车体约定相反，
   只改 CHASSIS_GYRO_Z_SIGN 为 -1。 */
#define CHASSIS_GYRO_Z_SIGN            1.0f
#define CHASSIS_GYRO_RATE_LPF          0.15f
#define CHASSIS_GYRO_RATE_KP            3.0f
#define CHASSIS_GYRO_CORR_MAX           0.35f
#define CHASSIS_GYRO_DEADBAND           0.01f
#define CHASSIS_STRAIGHT_W_THRESHOLD    0.03f

/* 正式上位机直行与模式 6 共用角度 P 环，不使用 gz 阻尼，也不加 Ki/Kd。
   原 IMU yaw = -atan2(...)，默认取反以统一为逆时针正。
   实车左转时模式 6 的 N 应增大；若减小，修改 YAW_SIGN。 */
#define CHASSIS_ANGLE_YAW_SIGN         (-1.0f)
#define CHASSIS_ANGLE_KP                 2.5f /* rad/s 每 rad 误差 */
#define CHASSIS_ANGLE_CORR_MAX           0.35f /* rad/s */
#define CHASSIS_ANGLE_DEADBAND_DEG       0.30f

/* 速度反馈一阶低通系数 (0~1, 越小越平滑但越迟钝)
   0.2 m/s 时每 5ms 只有约 2 个编码器计数, 量化噪声非常大,
   0.5 几乎不过滤, 导致 PWM 乱跳。改成 0.1, 相当于把约 10 个采样
   (50ms) 平均, 噪声降到约 1/3, PWM 明显平稳。代价是响应稍慢,
   对底盘速度环来说完全可接受。 */
#define CHASSIS_SPEED_LPF       0.1f

/* 硬件安装方向修正: 实测两轮都是"前进时计数为负", 所以两轮都取 -1。
   Encoder_SetDirSign(-1) 之后, 前进 left/right count 都为正。 */
#define CHASSIS_ENC_LEFT_SIGN   (-1)
#define CHASSIS_ENC_RIGHT_SIGN  (-1)

#define CHASSIS_PI              3.14159265358979f

/* ======================== 私有变量 ======================== */

static pid_type_def s_pid[ENC_COUNT];

static float s_target_mps[ENC_COUNT];
static float s_control_target_mps[ENC_COUNT];
static float s_fdb_mps[ENC_COUNT];
static float s_gyro_rate_filt;
static float s_gyro_corr_w;
static uint8_t s_straight_hold_en;
static uint8_t s_angle_hold_en;
/* 0=未锁定上位机直行，+1=前进段，-1=后退段。
   与测试模式分开，重复串口命令不重新锁定参考角度。 */
static int8_t s_host_straight_dir;
static float s_angle_ref_rad;
static float s_angle_now_rad;
static float s_angle_error_rad;
static float s_angle_corr_w;

static Chassis_Odom_t s_odom;

static uint8_t s_speed_loop_en;   /* 0 = 开环 PWM 调试, 1 = 速度闭环 */

/* 上电自检结果, 期望两路都约 400 个 A 相边沿 */
static int32_t s_selftest_l;
static int32_t s_selftest_r;

/* 模式 7 的诊断窗口状态。放在函数外，便于进入模式或长按按键时清零。 */
static uint32_t s_diag_last_tick;
static int32_t  s_diag_hz_l;
static int32_t  s_diag_hz_r;
static int32_t  s_diag_acc_l;
static int32_t  s_diag_acc_r;

/* 闭环测试期间的 PWM 上限 (安全钳位)。
   实测 PWM 20 -> 261 mm/s, 所以 1 个 PWM ≈ 0.013 m/s,
   闭环目标 0.2 m/s 只需约 15 PWM。若这里顶到限值还在加速,
   说明控制逻辑有问题, 而不是目标速度定得太快。
   只在测试模式(s_test_mode==2)下生效, 不影响正式运行。 */
#define CHASSIS_TEST_PID_LIMIT   30

/* 当前测试模式: 0=非测试, 1=开环, 2=速度环, 3=gz修正, 6=角度锁定 */
static uint8_t s_test_mode;

/* ======================== 私有函数 ======================== */

static float ClampF(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/**
 * @brief  把角度折算到 [-pi, pi]
 */
static float WrapPi(float a)
{
    while (a >  CHASSIS_PI) a -= (2.0f * CHASSIS_PI);
    while (a < -CHASSIS_PI) a += (2.0f * CHASSIS_PI);
    return a;
}

static float BhaskaraSinPositive(float x)
{
    float num = x * (CHASSIS_PI - x);

    return (16.0f * num) /
           (5.0f * CHASSIS_PI * CHASSIS_PI - 4.0f * num);
}

/**
 * @brief  使用 Bhaskara I 近似计算 sin/cos，输入角度归一化到 [-pi, pi]
 * @note   不依赖 libm；在 5ms 里程计积分中，近似误差足够小。
 */
static void SinCosApprox(float a, float *s, float *c)
{
    float x = WrapPi(a);
    float ax = (x < 0.0f) ? -x : x;
    float half_pi = 0.5f * CHASSIS_PI;
    float sin_abs = BhaskaraSinPositive(ax);

    *s = (x < 0.0f) ? -sin_abs : sin_abs;
    if (ax <= half_pi) {
        *c = BhaskaraSinPositive(half_pi - ax);
    } else {
        *c = -BhaskaraSinPositive(ax - half_pi);
    }
}

/**
 * @brief  归一化轮速 (正 = 前进) 转底层 Load() 原始量
 * @note   全工程唯一的符号转换点
 */
static void Chassis_OutputRaw(int left_norm, int right_norm)
{
    Load(-left_norm, -right_norm);
}

/* ======================== 公有函数 ======================== */

void Chassis_Init(void)
{
    Encoder_Init();

    Encoder_SetDirSign(ENC_LEFT,  (int8_t)CHASSIS_ENC_LEFT_SIGN);
    Encoder_SetDirSign(ENC_RIGHT, (int8_t)CHASSIS_ENC_RIGHT_SIGN);

    PID_init(&s_pid[ENC_LEFT],  PID_POSITION, s_pid_left_param,
             CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);
    PID_init(&s_pid[ENC_RIGHT], PID_POSITION, s_pid_right_param,
             CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);

    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
    s_control_target_mps[ENC_LEFT] = 0.0f;
    s_control_target_mps[ENC_RIGHT] = 0.0f;
    s_fdb_mps[ENC_LEFT]     = 0.0f;
    s_fdb_mps[ENC_RIGHT]    = 0.0f;
    s_gyro_rate_filt         = 0.0f;
    s_gyro_corr_w            = 0.0f;
    s_straight_hold_en       = 0u;
    s_angle_hold_en = 0u;
    s_host_straight_dir = 0;
    s_angle_ref_rad = 0.0f;
    s_angle_now_rad = 0.0f;
    s_angle_error_rad = 0.0f;
    s_angle_corr_w = 0.0f;

    s_odom.x_m       = 0.0f;
    s_odom.y_m       = 0.0f;
    s_odom.theta_rad = 0.0f;
    s_odom.v_mps     = 0.0f;
    s_odom.w_radps   = 0.0f;

    s_speed_loop_en = 0;   /* 默认开环, 标定完方向/分辨率再打开 */
}

void Chassis_Update(void)
{
    uint8_t i;
    float d_left_m;
    float d_right_m;
    float ds_m;
    float dtheta;
    float th_mid;
    float s_th;
    float c_th;

    /* 应用上位机最新速度指令 (0xBB 帧解析结果) */
    Host_ApplyPendingCommand();

    Encoder_Update();

    /* ---- 1. 轮速反馈 + 一阶低通 ---- */
    for (i = 0; i < ENC_COUNT; i++) {
        const Encoder_t *e = Encoder_Get((EncoderID_t)i);
        if (e == 0) {
            continue;
        }
        s_fdb_mps[i] += CHASSIS_SPEED_LPF * (e->velocity_mps - s_fdb_mps[i]);
    }

    /* IMU 数据由主循环采样，这里只读取最近值，不在定时器中断里访问 SPI。 */
    s_gyro_rate_filt += CHASSIS_GYRO_RATE_LPF *
        (CHASSIS_GYRO_Z_SIGN * IMU_GetYawRateRadps() - s_gyro_rate_filt);

    s_angle_now_rad = WrapPi(CHASSIS_ANGLE_YAW_SIGN * IMU_GetYawDeg()
                            * CHASSIS_PI / 180.0f);
    s_angle_corr_w = 0.0f;
    if ((s_angle_hold_en != 0u) && (s_speed_loop_en != 0u)) {
        /* 最短角度差处理 +/-180 度跨界，启动角度只锁定一次。 */
        s_angle_error_rad = WrapPi(s_angle_ref_rad - s_angle_now_rad);
        if (fabsf(s_angle_error_rad) > CHASSIS_ANGLE_DEADBAND_DEG * CHASSIS_PI / 180.0f) {
            s_angle_corr_w = ClampF(CHASSIS_ANGLE_KP * s_angle_error_rad,
                                   -CHASSIS_ANGLE_CORR_MAX, CHASSIS_ANGLE_CORR_MAX);
        }
    } else {
        s_angle_error_rad = 0.0f;
    }

    /* ---- 2. 里程计: 用原始增量, 不走低通, 免得抹掉细节 ---- */
    d_left_m  = (float)Encoder_GetDelta(ENC_LEFT)  * Encoder_GetMetersPerCount(ENC_LEFT);
    d_right_m = (float)Encoder_GetDelta(ENC_RIGHT) * Encoder_GetMetersPerCount(ENC_RIGHT);

    ds_m   = 0.5f * (d_left_m + d_right_m);
    dtheta = (d_right_m - d_left_m) / WHEEL_SEPARATION_M;

    /* 中点法积分: 比"先转后走"近似更准, 原地旋转也不退化 */
    th_mid = s_odom.theta_rad + 0.5f * dtheta;
    SinCosApprox(th_mid, &s_th, &c_th);

    s_odom.x_m      += ds_m * c_th;
    s_odom.y_m      += ds_m * s_th;
    s_odom.theta_rad = WrapPi(s_odom.theta_rad + dtheta);

    s_odom.v_mps   = 0.5f * (s_fdb_mps[ENC_LEFT] + s_fdb_mps[ENC_RIGHT]);
    s_odom.w_radps = (s_fdb_mps[ENC_RIGHT] - s_fdb_mps[ENC_LEFT]) / WHEEL_SEPARATION_M;

    /* ---- 3. 速度环 / 开环输出 ---- */
    s_gyro_corr_w = 0.0f;
    if (s_speed_loop_en != 0u) {
        int out[ENC_COUNT];
        uint8_t wheel;

        for (wheel = 0; wheel < ENC_COUNT; wheel++) {
            float target = s_target_mps[wheel];
            float error;
            float ff_gain;
            float control;
            float sign;

            if ((target > -0.001f) && (target < 0.001f)) {
                PID_clear(&s_pid[wheel]);
                s_pid[wheel].out = 0.0f;
                s_control_target_mps[wheel] = 0.0f;
                out[wheel] = 0;
                continue;
            }

            /* 正式直行与模式 6 使用角度环；模式 3 保留角速度外环。
               上位机转弯和原地旋转时关闭两种外环。 */
            if (s_angle_hold_en != 0u) {
                float half_w = 0.5f * WHEEL_SEPARATION_M * s_angle_corr_w;
                target += (wheel == ENC_LEFT) ? -half_w : half_w;
                target = ClampF(target, -CHASSIS_MAX_WHEEL_MPS, CHASSIS_MAX_WHEEL_MPS);
            } else if (s_straight_hold_en != 0u) {
                float half_w;
                if ((s_gyro_rate_filt > -CHASSIS_GYRO_DEADBAND) &&
                    (s_gyro_rate_filt < CHASSIS_GYRO_DEADBAND)) {
                    s_gyro_corr_w = 0.0f;
                } else {
                    s_gyro_corr_w = ClampF(-CHASSIS_GYRO_RATE_KP * s_gyro_rate_filt,
                                       -CHASSIS_GYRO_CORR_MAX,
                                       CHASSIS_GYRO_CORR_MAX);
                }
                half_w = 0.5f * WHEEL_SEPARATION_M * s_gyro_corr_w;
                target += (wheel == ENC_LEFT) ? -half_w : half_w;
                target = ClampF(target, -CHASSIS_MAX_WHEEL_MPS, CHASSIS_MAX_WHEEL_MPS);
            }

            s_control_target_mps[wheel] = target;

            error = target - s_fdb_mps[wheel];
            ff_gain = (wheel == ENC_LEFT) ? CHASSIS_LEFT_FF_PWM_PER_MPS
                                          : CHASSIS_RIGHT_FF_PWM_PER_MPS;
            sign = (target >= 0.0f) ? 1.0f : -1.0f;
            control = ff_gain * target + sign * CHASSIS_FF_STATIC_PWM
                    + PID_calc(&s_pid[wheel], s_fdb_mps[wheel], target)
                    ;
            control = ClampF(control, -CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_OUT);
            /* 板上模式 2 只用于低速验收，落实 30 PWM 的安全钳位。
               正式串口控制、模式 3/6 不受这个测试限值影响。 */
            if (s_test_mode == 2u) {
                control = ClampF(control, -(float)CHASSIS_TEST_PID_LIMIT,
                                 (float)CHASSIS_TEST_PID_LIMIT);
            }
            s_pid[wheel].out = control;
            out[wheel] = (int)control;
        }
        Chassis_OutputRaw(out[ENC_LEFT], out[ENC_RIGHT]);
    }
    /* 开环模式下 PWM 由 Chassis_SetWheelRaw() 直接写, 这里不覆盖 */
}

void Chassis_SetWheelSpeed(float left_mps, float right_mps)
{
    /* 直接轮速接口清除角度锁定；SetTwist 随后按直行段恢复或重新锁角。 */
    s_angle_hold_en = 0u;
    s_host_straight_dir = 0;
    s_target_mps[ENC_LEFT]  = ClampF(left_mps,  -CHASSIS_MAX_WHEEL_MPS, CHASSIS_MAX_WHEEL_MPS);
    s_target_mps[ENC_RIGHT] = ClampF(right_mps, -CHASSIS_MAX_WHEEL_MPS, CHASSIS_MAX_WHEEL_MPS);
    /* 直接使用左右轮接口时，仅把几乎相等的非零目标当作直行。 */
    s_straight_hold_en = ((fabsf(left_mps + right_mps) > 0.02f) &&
                          (fabsf(left_mps - right_mps) < CHASSIS_STRAIGHT_W_THRESHOLD)) ? 1u : 0u;
    s_speed_loop_en = 1u;
}

void Chassis_SetWheelRaw(int left, int right)
{
    s_angle_hold_en = 0u;
    s_host_straight_dir = 0;
    /* 切回开环, 并把 PID 积分清掉, 免得下次闭环时带着旧积分冲一下 */
    s_speed_loop_en = 0u;
    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
    s_control_target_mps[ENC_LEFT] = 0.0f;
    s_control_target_mps[ENC_RIGHT] = 0.0f;
    s_straight_hold_en = 0u;
    s_gyro_corr_w = 0.0f;
    PID_clear(&s_pid[ENC_LEFT]);
    PID_clear(&s_pid[ENC_RIGHT]);

    if (left  >  TB6612_PWM_MAX) left  =  TB6612_PWM_MAX;
    if (left  <  TB6612_PWM_MIN) left  =  TB6612_PWM_MIN;
    if (right >  TB6612_PWM_MAX) right =  TB6612_PWM_MAX;
    if (right <  TB6612_PWM_MIN) right =  TB6612_PWM_MIN;

    Chassis_OutputRaw(left, right);
}

void Chassis_SetTwist(float v_mps, float w_radps)
{
    /* 串口量化到 0.001 rad/s：任何非零转向命令都交给上位机，
       不吞掉导航控制器的小角速度修正。 */
    int8_t straight_dir = 0;
    uint8_t keep_reference;
    float half_w = 0.5f * WHEEL_SEPARATION_M * w_radps;

    if ((fabsf(v_mps) >= 0.001f) && (fabsf(w_radps) < 0.0005f)) {
        straight_dir = (v_mps > 0.0f) ? 1 : -1;
    }
    keep_reference = ((s_angle_hold_en != 0u) &&
                      (s_host_straight_dir == straight_dir) &&
                      (straight_dir != 0)) ? 1u : 0u;

    if ((fabsf(v_mps) < 0.001f) && (fabsf(w_radps) < 0.0005f)) {
        Chassis_Stop();
        return;
    }

    Chassis_SetWheelSpeed(v_mps - half_w, v_mps + half_w);
    s_straight_hold_en = 0u; /* 正式控制不使用模式 3 的 gz 阻尼。 */
    s_angle_corr_w = 0.0f;
    s_angle_error_rad = 0.0f;
    if (straight_dir != 0) {
        if (keep_reference == 0u) {
            s_angle_ref_rad = WrapPi(CHASSIS_ANGLE_YAW_SIGN * IMU_GetYawDeg()
                                    * CHASSIS_PI / 180.0f);
        }
        s_host_straight_dir = straight_dir;
        s_angle_hold_en = 1u;
    }
}

void Chassis_Stop(void)
{
    s_angle_hold_en = 0u;
    s_host_straight_dir = 0;
    s_angle_corr_w = 0.0f;
    s_angle_error_rad = 0.0f;
    s_speed_loop_en = 0u;
    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
    s_control_target_mps[ENC_LEFT] = 0.0f;
    s_control_target_mps[ENC_RIGHT] = 0.0f;
    s_straight_hold_en = 0u;
    s_gyro_corr_w = 0.0f;
    PID_clear(&s_pid[ENC_LEFT]);
    PID_clear(&s_pid[ENC_RIGHT]);
    Load(0, 0);
}

void Chassis_Brake(void)
{
    s_angle_hold_en = 0u;
    s_host_straight_dir = 0;
    s_angle_corr_w = 0.0f;
    s_angle_error_rad = 0.0f;
    s_speed_loop_en = 0u;
    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
    s_straight_hold_en = 0u;
    s_gyro_corr_w = 0.0f;
    PID_clear(&s_pid[ENC_LEFT]);
    PID_clear(&s_pid[ENC_RIGHT]);
    Motor_BrakeAll();
}

const Chassis_Odom_t *Chassis_GetOdom(void)
{
    return &s_odom;
}

void Chassis_ResetOdom(void)
{
    Encoder_Reset();

    s_odom.x_m       = 0.0f;
    s_odom.y_m       = 0.0f;
    s_odom.theta_rad = 0.0f;
    s_odom.v_mps     = 0.0f;
    s_odom.w_radps   = 0.0f;
}

float Chassis_GetTarget(EncoderID_t wheel)
{
    if ((uint8_t)wheel >= ENC_COUNT) {
        return 0.0f;
    }
    return s_target_mps[wheel];
}

float Chassis_GetGyroRateRadps(void)
{
    return s_gyro_rate_filt;
}

float Chassis_GetGyroCorrectionRadps(void)
{
    return s_gyro_corr_w;
}

uint8_t Chassis_IsStraightHoldEnabled(void)
{
    return s_straight_hold_en;
}

uint8_t Chassis_IsAngleHoldEnabled(void)
{
    return s_angle_hold_en;
}

void Chassis_DebugDisplayGyro(void)
{
    int raw_mradps = (int)(IMU_GetYawRateRadps() * 1000.0f);
    int gz_mradps = (int)(s_gyro_rate_filt * 1000.0f);
    int corr_mradps = (int)(s_gyro_corr_w * 1000.0f);
    const Encoder_t *el = Encoder_Get(ENC_LEFT);
    const Encoder_t *er = Encoder_Get(ENC_RIGHT);

    OLED_operate_gram(PEN_CLEAR);

    OLED_show_string(1, 0, (uint8_t*)"raw gz:");
    OLED_printf(1, 8, "%d", raw_mradps);
    OLED_show_string(2, 0, (uint8_t*)"ctrl gz:");
    OLED_printf(2, 9, "%d", gz_mradps);
    OLED_show_string(3, 0, (uint8_t*)"corr:");
    OLED_printf(3, 6, "%d", corr_mradps);
    OLED_show_string(4, 0, (uint8_t*)"straight:");
    OLED_printf(4, 10, "%d", (int)s_straight_hold_en);
    OLED_refresh_gram();
}

/**
 * @brief  标定阶段显示
 * @note   用工程自带的 OLED 驱动, 全部通过 OLED_show_string / OLED_printf 输出。
 *
 *         注意: 工程开了 MicroLIB, 它不支持 %f 浮点格式化, 直接传
 *         "%.0f" / "%.3f" 会把格式串原样打在屏上。所以这里统一
 *         先乘 1000 转成整数, 再用 %d 加手工小数点的方式显示。
 *
 *         行 1: 左右累计计数
 *         行 2: 左右轮速 (mm/s)
 *         行 3: 左右累计距离 (mm)
 *         行 4: 里程计 x (m) 与 theta (rad)
 */
void Chassis_DebugDisplay(void)
{
    const Encoder_t *el = Encoder_Get(ENC_LEFT);
    const Encoder_t *er = Encoder_Get(ENC_RIGHT);

    OLED_operate_gram(PEN_CLEAR);

    OLED_show_string(1, 0, (uint8_t*)"cnt  :");
    OLED_printf(1, 7, "%d,%d", (int)(el->count), (int)(er->count));

    OLED_show_string(2, 0, (uint8_t*)"v mm/s:");
    OLED_printf(2, 7, "%d,%d",
                (int)(el->velocity_mps * 1000.0f),
                (int)(er->velocity_mps * 1000.0f));

    OLED_show_string(3, 0, (uint8_t*)"d mm :");
    OLED_printf(3, 7, "%d,%d",
                (int)(el->distance_m * 1000.0f),
                (int)(er->distance_m * 1000.0f));

    /* 行 4 右值是 mrad (乘以 1000 的弧度) */
    OLED_show_string(4, 0, (uint8_t*)"x mm :");
    OLED_printf(4, 7, "%d,%d",
                (int)(s_odom.x_m * 1000.0f),
                (int)(s_odom.theta_rad * 1000.0f));

    OLED_refresh_gram();
}

/*******************************************************************************
 *                     速度环测试 (开环/闭环切换)
 ******************************************************************************/

/* 闭环测试目标轮速 (m/s), 正 = 前进。
   0.2 m/s 在 5ms 采样周期下有约 3.8 个编码器计数, 量化误差 ~26%;
   再低(如 0.1)会掉到 1.9 个计数, 误差接近 50%, 速度环会明显发抖。
   所以自动控制的最低建议速度是 0.2 m/s。 */
#define CHASSIS_TEST_SPEED_MPS   0.20f

/* 开环基准 PWM。
   实测: Load(-20,-20) 时轮速 act = 261 mm/s, 是你觉得"舒服"的速度。
   由此 1 个 PWM ≈ 0.013 m/s, 闭环目标 0.2 m/s 只需要约 15 PWM。 */
#define CHASSIS_TEST_OPEN_PWM    20

void Chassis_TestSpeedStart(void)
{
    PID_clear(&s_pid[ENC_LEFT]);
    PID_clear(&s_pid[ENC_RIGHT]);
    Encoder_Reset();
    s_test_mode = 2u;
    Chassis_SetWheelSpeed(CHASSIS_TEST_SPEED_MPS, CHASSIS_TEST_SPEED_MPS);
    /* 模式 2 专门验证编码器速度环，禁止陀螺仪外环。 */
    s_straight_hold_en = 0u;
}

void Chassis_TestGyroStraightStart(void)
{
    PID_clear(&s_pid[ENC_LEFT]);
    PID_clear(&s_pid[ENC_RIGHT]);
    Encoder_Reset();
    s_test_mode = 3u;
    /* 正的归一化轮速最终由 Chassis_OutputRaw() 翻译成 Load(-L,-R)。 */
    /* 模式 3 保留 gz 阻尼对比，正式 SetTwist 已使用角度环。 */
    Chassis_SetWheelSpeed(CHASSIS_TEST_SPEED_MPS, CHASSIS_TEST_SPEED_MPS);
    s_straight_hold_en = 1u;
}

void Chassis_TestAngleStraightStart(void)
{
    Chassis_Stop();
    Encoder_Reset();
    Chassis_SetWheelSpeed(CHASSIS_TEST_SPEED_MPS, CHASSIS_TEST_SPEED_MPS);
    s_straight_hold_en = 0u;
    s_angle_ref_rad = WrapPi(CHASSIS_ANGLE_YAW_SIGN * IMU_GetYawDeg()
                            * CHASSIS_PI / 180.0f);
    s_angle_now_rad = s_angle_ref_rad;
    s_angle_error_rad = 0.0f;
    s_angle_corr_w = 0.0f;
    s_test_mode = 6u;
    s_angle_hold_en = 1u;
}

void Chassis_DebugDisplayAngleTest(void)
{
    /* 角度乘 10 显示：123 = 12.3 度；C 为 mrad/s。
       整页画完再刷屏，避免旧字符残留。 */
    OLED_operate_gram(PEN_CLEAR);
    OLED_printf(1, 0, "%s T/N:%d,%d", (s_host_straight_dir != 0) ? "AH" : "A6",
                (int)(s_angle_ref_rad * 1800.0f / CHASSIS_PI),
                (int)(s_angle_now_rad * 1800.0f / CHASSIS_PI));
    OLED_printf(2, 0, "E/C:%d,%d", (int)(s_angle_error_rad * 1800.0f / CHASSIS_PI),
                (int)(s_angle_corr_w * 1000.0f));
    OLED_printf(3, 0, "act:%d,%d", (int)(s_fdb_mps[ENC_LEFT] * 1000.0f),
                (int)(s_fdb_mps[ENC_RIGHT] * 1000.0f));
    OLED_printf(4, 0, "pwm:%d,%d", (int)s_pid[ENC_LEFT].out, (int)s_pid[ENC_RIGHT].out);
    OLED_refresh_gram();
}

/**
 * @brief  开环基准: 用你原来 Load(-20,-20) 的 PWM 值跑
 * @note   实测这个 PWM 对应轮速 261 mm/s。
 *         闭环目标 0.2 m/s 应该只需要约 15 PWM, 所以若闭环顶到限幅,
 *         就说明控制逻辑有问题, 而不是"目标定得太快"。
 */
void Chassis_TestOpenLoopStart(void)
{
    Encoder_Reset();
    s_test_mode = 1u;
    Chassis_SetWheelRaw(CHASSIS_TEST_OPEN_PWM, CHASSIS_TEST_OPEN_PWM);
}

void Chassis_TestStop(void)
{
    s_test_mode = 0u;
    Chassis_Stop();
}

/**
 * @brief  显示速度环/开环测试
 * @note   行 1: 目标轮速 (mm/s); 开环模式下显示固定 PWM
 *         行 2: 实测轮速 (mm/s)  ← 关键, 对比两种模式这行是否一致
 *         行 3: 累计计数
 *         行 4: 实际加在电机上的 PWM
 *
 *         对比方法:
 *           开环 PWM=20 时, 看 act 是多少 (这就是"舒服速度"的真实值)
 *           闭环 tgt=200 时, 看 pwm 是多少、act 是否到 200
 *         若开环 20 的 act 已经接近 200, 说明 0.2 m/s 本来就只需要 ~20 PWM,
 *         闭环却顶到 100, 那就是 PID 参数或符号有问题。
 */
void Chassis_DebugDisplaySpeedTest(void)
{
    const Encoder_t *el = Encoder_Get(ENC_LEFT);
    const Encoder_t *er = Encoder_Get(ENC_RIGHT);
    int pwm_l;
    int pwm_r;

    OLED_operate_gram(PEN_CLEAR);

    if (s_test_mode == 1u) {
        pwm_l = CHASSIS_TEST_OPEN_PWM;
        pwm_r = CHASSIS_TEST_OPEN_PWM;
        OLED_show_string(1, 0, (uint8_t*)"OPEN pwm:");
        OLED_printf(1, 9, "%d", CHASSIS_TEST_OPEN_PWM);
    } else {
        pwm_l = (int)s_pid[ENC_LEFT].out;
        pwm_r = (int)s_pid[ENC_RIGHT].out;
        OLED_show_string(1, 0, (uint8_t*)((s_test_mode == 3u) ? "GYRO tgt:" : "SPEED tgt:"));
        OLED_printf(1, 11, "%d,%d",
                    (int)(s_control_target_mps[ENC_LEFT] * 1000.0f),
                    (int)(s_control_target_mps[ENC_RIGHT] * 1000.0f));
    }

    OLED_show_string(2, 0, (uint8_t*)"act :");
    OLED_printf(2, 7, "%d,%d",
                (int)(el->velocity_mps * 1000.0f),
                (int)(er->velocity_mps * 1000.0f));

    OLED_show_string(3, 0, (uint8_t*)"cnt :");
    OLED_printf(3, 7, "%d,%d", (int)(el->count), (int)(er->count));

    OLED_show_string(4, 0, (uint8_t*)"pwm :");
    OLED_printf(4, 7, "%d,%d", pwm_l, pwm_r);

    OLED_refresh_gram();
}

/**
 * @brief  上电自检: 软件翻转 A 相, 走完整条"捕获->中断->计数"通路
 * @note   200 次翻转 = 200 个上升沿 + 200 个下降沿 = 400 次捕获中断。
 *         自检返回捕获边沿总数，期望约为 400；固定 B 相时净方向计数会抵消。
 */
void Chassis_SelfTest(void)
{
    OLED_operate_gram(PEN_CLEAR);
    OLED_show_string(1, 0, (uint8_t*)"Encoder SelfTest");
    OLED_show_string(2, 0, (uint8_t*)"toggling A phase");
    OLED_show_string(3, 0, (uint8_t*)"please wait...  ");
    OLED_refresh_gram();

    s_selftest_l = Encoder_SelfTest(ENC_LEFT,  200);
    s_selftest_r = Encoder_SelfTest(ENC_RIGHT, 200);

    /* 自检动过原始计数, 清零重来, 免得污染后面的标定 */
    Encoder_Reset();
}

/**
 * @brief  显示自检结果
 * @note   期望 L/R 都约等于 400。
 *         L 或 R 为 0 -> 那一路的捕获通路有问题(代码/时钟/NVIC), 与外部信号无关
 *         L 和 R 都约 400 -> 代码通路正常, 轮子不动就是外部信号的问题
 */
void Chassis_DebugDisplaySelfTest(void)
{
    OLED_operate_gram(PEN_CLEAR);
    OLED_show_string(1, 0, (uint8_t*)"SELFTEST");
    OLED_show_string(2, 0, (uint8_t*)"expect ~400");
    OLED_show_string(3, 0, (uint8_t*)"L,R   :");
    OLED_printf(3, 7, "%d,%d", (int)s_selftest_l, (int)s_selftest_r);
    OLED_show_string(4, 0, (uint8_t*)"press KEY->run");
    OLED_refresh_gram();
}

/**
 * @brief  诊断显示: 原始捕获中断频率 + 累计计数
 * @note   ISR 值 = 自上次刷新以来(0.5s) 两个轮子的捕获中断次数, 折算成 Hz。
 *         它直接反映"引脚上每秒有多少个边沿", 完全绕开判方向和累加逻辑,
 *         所以能把"信号问题"和"计数逻辑问题"分开:
 *
 *           轮子静止、电机断电     -> L/R 都应为 0
 *           手转轮子 1 圈/秒       -> 约 780 Hz (13线 x 2边沿 x 30减速比)
 *           "扭一下就上万" 时看这里 -> 若是几百 kHz, 是信号/中断问题;
 *                                     若只有几十 Hz, 是判方向/累加逻辑问题
 */
void Chassis_DebugDisplayIsr(void)
{
    float theta_total_deg;

    s_diag_acc_l += Encoder_TakeIsrCount(ENC_LEFT);
    s_diag_acc_r += Encoder_TakeIsrCount(ENC_RIGHT);

    /* nowtime 单位 100us, 5000 = 0.5 秒 */
    if ((nowtime - s_diag_last_tick) >= 5000u) {
        s_diag_hz_l     = s_diag_acc_l * 2;   /* 0.5s 窗口 x2 = Hz */
        s_diag_hz_r     = s_diag_acc_r * 2;
        s_diag_acc_l    = 0;
        s_diag_acc_r    = 0;
        s_diag_last_tick = nowtime;
    }

    OLED_operate_gram(PEN_CLEAR);

    OLED_show_string(1, 0, (uint8_t*)"ISR L,R :");
    OLED_printf(1, 10, "%d,%d", (int)s_diag_hz_l, (int)s_diag_hz_r);

    OLED_show_string(2, 0, (uint8_t*)"cnt L,R :");
    OLED_printf(2, 10, "%d,%d", (int)Encoder_GetCount(ENC_LEFT),
                                (int)Encoder_GetCount(ENC_RIGHT));

    OLED_show_string(3, 0, (uint8_t*)"d mm    :");
    OLED_printf(3, 10, "%d,%d",
                (int)(Encoder_Get(ENC_LEFT)->distance_m * 1000.0f),
                (int)(Encoder_Get(ENC_RIGHT)->distance_m * 1000.0f));

    /* 从累计轮计数计算不回绕角度，供原地旋转多圈标定有效轮距。
       s_odom.theta_rad 会限制在 +/-180 度，不能用于多圈标定。 */
    theta_total_deg = (
        (float)Encoder_GetCount(ENC_RIGHT) * Encoder_GetMetersPerCount(ENC_RIGHT)
      - (float)Encoder_GetCount(ENC_LEFT)  * Encoder_GetMetersPerCount(ENC_LEFT))
      / WHEEL_SEPARATION_M * 180.0f / CHASSIS_PI;

    OLED_show_string(4, 0, (uint8_t*)"th deg  :");
    OLED_printf(4, 10, "%d", (int)theta_total_deg);

    OLED_refresh_gram();
}

void Chassis_DebugResetIsr(void)
{
    Chassis_ResetOdom();
    s_diag_last_tick = nowtime;
    s_diag_hz_l      = 0;
    s_diag_hz_r      = 0;
    s_diag_acc_l     = 0;
    s_diag_acc_r     = 0;

    /* Encoder_Reset 已清零边沿计数；再取一次，避免以后实现变化留下旧样本。 */
    (void)Encoder_TakeIsrCount(ENC_LEFT);
    (void)Encoder_TakeIsrCount(ENC_RIGHT);
}
