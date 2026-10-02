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
 * 【已由用户调定】Ki = 0, 只用 Kp + Kd:
 *   Kp = 100 : 有劲、响应快
 *   Kd = 5   : 阻尼, 抑制超调(小值避免放大编码器量化噪声)
 *   Ki = 0   : 纯 PD, 有稳态误差(实际约 104 mm/s, 到不了 200), 属正常。
 */
static const fp32 s_pid_left_param[3]  = { 100.0f, 0.0f, 5.0f };
static const fp32 s_pid_right_param[3] = { 100.0f, 0.0f, 5.0f };
#define CHASSIS_PID_MAX_OUT    100.0f
#define CHASSIS_PID_MAX_IOUT   0.0f

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
static float s_fdb_mps[ENC_COUNT];

static Chassis_Odom_t s_odom;

static uint8_t s_speed_loop_en;   /* 0 = 开环 PWM 调试, 1 = 速度闭环 */

/* 上电自检结果, 期望两路都约 +100 */
static int32_t s_selftest_l;
static int32_t s_selftest_r;

/* 闭环测试期间的 PWM 上限 (安全钳位)。
   实测 PWM 20 -> 261 mm/s, 所以 1 个 PWM ≈ 0.013 m/s,
   闭环目标 0.2 m/s 只需约 15 PWM。若这里顶到限值还在加速,
   说明控制逻辑有问题, 而不是目标速度定得太快。
   只在测试模式(s_test_mode==2)下生效, 不影响正式运行。 */
#define CHASSIS_TEST_PID_LIMIT   30

/* 当前测试模式: 0=非测试, 1=开环基准, 2=闭环速度环 */
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

/**
 * @brief  sin/cos 近似 (Bhaskara I), 误差 < 0.2%
 * @note   本工程没链接 libm, 所以不用 sinf/cosf。
 *         里程计是 5ms 小增量积分, 0.2% 的三角函数误差完全可以忽略。
 */
static void SinCosApprox(float a, float *s, float *c)
{
    float x = WrapPi(a);
    float sgn = 1.0f;
    float num;

    if (x < 0.0f) {
        x = -x;
        sgn = -1.0f;
    }

    /* x 在 [0, pi], sin(x) = 4x(pi-x) / (pi^2 + x(pi-x)) */
    num = x * (CHASSIS_PI - x);
    *s = sgn * (4.0f * num) / (CHASSIS_PI * CHASSIS_PI + num);
    *c = (CHASSIS_PI * CHASSIS_PI - 4.0f * num) / (CHASSIS_PI * CHASSIS_PI + num);
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
    s_fdb_mps[ENC_LEFT]     = 0.0f;
    s_fdb_mps[ENC_RIGHT]    = 0.0f;

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

    /* ---- 2. 里程计: 用原始增量, 不走低通, 免得抹掉细节 ---- */
    d_left_m  = (float)Encoder_GetDelta(ENC_LEFT)  * ENC_METERS_PER_COUNT;
    d_right_m = (float)Encoder_GetDelta(ENC_RIGHT) * ENC_METERS_PER_COUNT;

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
    if (s_speed_loop_en != 0u) {
        Chassis_OutputRaw(
            (int)PID_calc(&s_pid[ENC_LEFT],  s_fdb_mps[ENC_LEFT],  s_target_mps[ENC_LEFT]),
            (int)PID_calc(&s_pid[ENC_RIGHT], s_fdb_mps[ENC_RIGHT], s_target_mps[ENC_RIGHT]));
    }
    /* 开环模式下 PWM 由 Chassis_SetWheelRaw() 直接写, 这里不覆盖 */
}

void Chassis_SetWheelSpeed(float left_mps, float right_mps)
{
    s_target_mps[ENC_LEFT]  = ClampF(left_mps,  -CHASSIS_MAX_WHEEL_MPS, CHASSIS_MAX_WHEEL_MPS);
    s_target_mps[ENC_RIGHT] = ClampF(right_mps, -CHASSIS_MAX_WHEEL_MPS, CHASSIS_MAX_WHEEL_MPS);
    s_speed_loop_en = 1u;
}

void Chassis_SetWheelRaw(int left, int right)
{
    /* 切回开环, 并把 PID 积分清掉, 免得下次闭环时带着旧积分冲一下 */
    s_speed_loop_en = 0u;
    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
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
    float half_w = 0.5f * WHEEL_SEPARATION_M * w_radps;
    Chassis_SetWheelSpeed(v_mps - half_w, v_mps + half_w);
}

void Chassis_Stop(void)
{
    s_speed_loop_en = 0u;
    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
    PID_clear(&s_pid[ENC_LEFT]);
    PID_clear(&s_pid[ENC_RIGHT]);
    Load(0, 0);
}

void Chassis_Brake(void)
{
    s_speed_loop_en = 0u;
    s_target_mps[ENC_LEFT]  = 0.0f;
    s_target_mps[ENC_RIGHT] = 0.0f;
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

    if (s_test_mode == 1u) {
        pwm_l = CHASSIS_TEST_OPEN_PWM;
        pwm_r = CHASSIS_TEST_OPEN_PWM;
        OLED_show_string(1, 0, (uint8_t*)"OPEN pwm:");
        OLED_printf(1, 9, "%d", CHASSIS_TEST_OPEN_PWM);
    } else {
        pwm_l = (int)s_pid[ENC_LEFT].out;
        pwm_r = (int)s_pid[ENC_RIGHT].out;
        OLED_show_string(1, 0, (uint8_t*)"tgt :");
        OLED_printf(1, 7, "%d,%d",
                    (int)(s_target_mps[ENC_LEFT] * 1000.0f),
                    (int)(s_target_mps[ENC_RIGHT] * 1000.0f));
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
 *         期望计数 ≈ +100 (A!=B 判正转, 见 Encoder_CaptureIRQ 的判据)。
 */
void Chassis_SelfTest(void)
{
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
 * @note   期望 L/R 都约等于 +100。
 *         L 或 R 为 0 -> 那一路的捕获通路有问题(代码/时钟/NVIC), 与外部信号无关
 *         L 和 R 都约 +100 -> 代码通路正常, 轮子不动就是外部信号的问题
 */
void Chassis_DebugDisplaySelfTest(void)
{
    OLED_show_string(1, 0, (uint8_t*)"SELFTEST");
    OLED_show_string(2, 0, (uint8_t*)"expect ~100");
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
 *           手转轮子 1 圈/秒       -> 约 26 Hz (13线 x 2边沿)
 *           "扭一下就上万" 时看这里 -> 若是几百 kHz, 是信号/中断问题;
 *                                     若只有几十 Hz, 是判方向/累加逻辑问题
 */
void Chassis_DebugDisplayIsr(void)
{
    static uint32_t last_ms;
    static int32_t  hz_l;
    static int32_t  hz_r;
    static int32_t  acc_l;
    static int32_t  acc_r;

    acc_l += Encoder_TakeIsrCount(ENC_LEFT);
    acc_r += Encoder_TakeIsrCount(ENC_RIGHT);

    /* nowtime 单位 100us, 5000 = 0.5 秒 */
    if ((nowtime - last_ms) >= 5000u) {
        hz_l    = acc_l * 2;   /* 0.5s 窗口 x2 = Hz */
        hz_r    = acc_r * 2;
        acc_l   = 0;
        acc_r   = 0;
        last_ms = nowtime;
    }

    OLED_show_string(1, 0, (uint8_t*)"ISR L,R :");
    OLED_printf(1, 10, "%d,%d", (int)hz_l, (int)hz_r);

    OLED_show_string(2, 0, (uint8_t*)"cnt L,R :");
    OLED_printf(2, 10, "%d,%d", (int)Encoder_GetCount(ENC_LEFT),
                                (int)Encoder_GetCount(ENC_RIGHT));

    OLED_show_string(3, 0, (uint8_t*)"d mm    :");
    OLED_printf(3, 10, "%d,%d",
                (int)(Encoder_Get(ENC_LEFT)->distance_m * 1000.0f),
                (int)(Encoder_Get(ENC_RIGHT)->distance_m * 1000.0f));

    OLED_show_string(4, 0, (uint8_t*)"x,th x1e3:");
    OLED_printf(4, 10, "%d,%d",
                (int)(s_odom.x_m * 1000.0f),
                (int)(s_odom.theta_rad * 1000.0f));

    OLED_refresh_gram();
}
