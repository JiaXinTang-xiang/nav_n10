#include "Xunji_task.h"

int RRR, RR, R, MR, ML, L, LL, LLL;
float Xunji_Left, Xunji_Right;    // 调试用：记录最后一次输出的左右速度
int32_t left_pwm=15, right_pwm=15;
int32_t LeftOut, RightOut;
int Xunji_State=0;

#define CORNER_DELAY  120     /* 延迟直行(周期数, 50Hz时120=600ms) */
/*
 * WDNL — 传感器电平判断宏（移植时只改这里）
 *   当前硬件 0=黑(线上) 非0=白(地上): #define WDNL(x) ((x) == 0)
 *   如果换成 非0=黑 0=白:          #define WDNL(x) ((x) != 0)
 */
#define WDNL(x)  ((x) == 0)

int Corner_Check(void);
static float ReadLinePosition(void);
static int Corner_Handler(int32_t crnr, int32_t l_in, int32_t r_in,
                           int32_t *l_out, int32_t *r_out);



int sensorPID(void)                                          // 无参数，返回偏差值
{
    int error = 0;                                           // 默认为0（居中），如果没有匹配项（丢线）就走直线
    RRR = HAL_GPIO_ReadPin(RRR_GPIO_Port, RRR_Pin);
    RR  = HAL_GPIO_ReadPin(RR_GPIO_Port, RR_Pin);
    R   = HAL_GPIO_ReadPin(R_GPIO_Port, R_Pin);
    MR  = HAL_GPIO_ReadPin(MR_GPIO_Port, MR_Pin);
    ML  = HAL_GPIO_ReadPin(ML_GPIO_Port, ML_Pin);
    L   = HAL_GPIO_ReadPin(L_GPIO_Port, L_Pin);
    LL  = HAL_GPIO_ReadPin(LL_GPIO_Port, LL_Pin);
    LLL = HAL_GPIO_ReadPin(LLL_GPIO_Port, LLL_Pin);

    if      (WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 14;  /* LLL */
    else if (WDNL(LLL)&&WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 12;   /* LLL+LL */
    else if (!WDNL(LLL)&&WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 10;   /* LL */
    else if (!WDNL(LLL)&&WDNL(LL)&&WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 8;   /* LL+L */
    else if (!WDNL(LLL)&&!WDNL(LL)&&WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 6;   /* L */
    else if (!WDNL(LLL)&&!WDNL(LL)&&WDNL(L)&&WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 4;   /* L+ML */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 2;   /* ML */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&WDNL(ML)&&WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = 0;   /* ML+MR 居中 */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = -2;  /* MR */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&WDNL(MR)&&WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = -4;  /* MR+R */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&WDNL(R)&&!WDNL(RR)&&!WDNL(RRR)) error = -6;  /* R */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&WDNL(R)&&WDNL(RR)&&!WDNL(RRR)) error = -8;  /* R+RR */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&WDNL(RR)&&!WDNL(RRR)) error = -10;  /* RR */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&WDNL(RR)&&WDNL(RRR)) error = -12;  /* RR+RRR */
    else if (!WDNL(LLL)&&!WDNL(LL)&&!WDNL(L)&&!WDNL(ML)&&!WDNL(MR)&&!WDNL(R)&&!WDNL(RR)&&WDNL(RRR)) error = -14; /* RRR */

    return error;
}

void xunji_PID(int32_t l_in, int32_t r_in, int32_t *l_out, int32_t *r_out)
{
    static float last_pos = 3.5f;                             // 上一次的有效位置（丢线时复用）
    static pid_type_def PID_Line;                             // 循迹闭环专用 PID（不是速度环那个）
    static uint8_t init_done = 0;                             // 一次性初始化标志

    if (!init_done) {
        float  pid_params[3] = {4.0f, 0.0f, 25.0f};   /* Kp Ki Kd 纯 PD 控制 */
        PID_init(&PID_Line, PID_POSITION, pid_params, 70.0f, 50.0f);
        init_done = 1;
    }

    sensorPID();
    int crnr = Corner_Check();
    int ret  = Corner_Handler(crnr, l_in, r_in, l_out, r_out);

     if (ret == 0) {							// 正常巡线
        // 全白（丢线）：ReadLinePosition 返回 -1.0 → pos < 0 成立 → 用上次位置
        // 全黑（十字/终点）：返回 -2.0 → 也用上次位置
        float pos = ReadLinePosition();         // 加权平均算出黑线位置（0.0~7.0）
        if (pos < 0.0f) pos = last_pos;         // 丢线时用上一次位置
        else last_pos = pos;                    // 正常时更新 last_pos

        // ref = pos - 3.5f：当前偏差（正=偏右，负=偏左）
        // set = 0.0f：目标是偏差为 0（线在正中间）
        int32_t diff = (int32_t)PID_calc(&PID_Line, pos - 3.5f, 0.0f);
        *l_out = l_in - diff;
        *r_out = r_in + diff;
     }
    else if (ret == 2) {                   // 刚退出直角
         PID_clear(&PID_Line);             // 清积分, 避免转弯后车尾摆动
     }

    Xunji_Left  = *l_out;
    Xunji_Right = *r_out;
}


/* ---------- 加权平均位置(闭环用) ---------- */
static float ReadLinePosition(void)                           // static = 只在本文件内可见
{
    int s[8];                                                 // 8个元素的数组，0=黑 1=白
    // 把传感器读数转为 0/1：WDNL成立=1(黑)，不成立=0(白)
    s[0] = WDNL(LLL); s[1] = WDNL(LL); s[2] = WDNL(L); s[3] = WDNL(ML);
    s[4] = WDNL(MR); s[5] = WDNL(R); s[6] = WDNL(RR); s[7] = WDNL(RRR);

    int cnt = s[0]+s[1]+s[2]+s[3]+s[4]+s[5]+s[6]+s[7];      // 有几颗传感器看到黑线
    if (cnt == 0) return -1.0f;   // 全白 = 完全丢线 → 返回 -1
    if (cnt == 8) return -2.0f;   // 全黑 = 到了交叉/终点 → 返回 -2

    // 加权平均：每颗传感器的权重 = 它的位置索引
    // 索引:  LLL=0  LL=1  L=2  ML=3  MR=4  R=5  RR=6  RRR=7
    float sum = (float)(s[0]*0 + s[1]*1 + s[2]*2 + s[3]*3 +
                        s[4]*4 + s[5]*5 + s[6]*6 + s[7]*7);
    return sum / cnt;                                         // 加权平均值 = 黑线的中心位置（0.0~7.0，中心是3.5）
}


/**
 * Corner_Check() — 直角检测(sensorPID之后调用)
 *
 * 左直角: 左边4路(LLL LL L ML)≥3路黑 → 返回 -1
 * 右直角: 右边4路(MR R RR RRR)≥3路黑 → 返回  1
 * 无直角: 返回0
 */
int Corner_Check(void)
{
    /* sensorPID里已更新过RRR~LLL, 直接读全局变量 */
    int l = WDNL(LLL) + WDNL(LL) + WDNL(L) + WDNL(ML);  // 统计左边4路有多少颗看到了黑线
    int r = WDNL(MR) + WDNL(R) + WDNL(RR) + WDNL(RRR);  // 统计右边4路有多少颗看到了黑线

    if (l >= 3) return -1;   // 左直角: 左边4路(LLL LL L ML)≥3路黑 → 返回 -1
    if (r >= 3) return  1;   // 右直角: 右边4路(MR R RR RRR)≥3路黑 → 返回  1
    return 0;                // 无直角: 返回0
}

/**
 * Corner_Handler — 直角处理：延迟直行 + 原地转弯(传感器检测退出)
 *
 * 两个循迹函数 xunji()/xunji_PID() 共用, static 变量在函数内部保存状态。
 * 因为每次 ISR 只调用其中一个(RunFlag 互斥), 共享状态是安全的。
 *
 * @param crnr   Corner_Check() 的结果
 * @param l_in/r_in 基础速度
 * @param l_out/r_out 输出速度(直角处理时会设置)
 * @return  0=正常巡线(调用者自己算差速)
 *          1=直角处理中(l_out/r_out已设置)
 *          2=刚退出直角(调用者正常循迹+清理积分)
 */
static int Corner_Handler(int32_t crnr, int32_t l_in, int32_t r_in,
                           int32_t *l_out, int32_t *r_out)
{
    static uint8_t  c_state = 0;                              // 直角状态: 0=空闲, 1=延迟直行, 2=原地转弯
    static int32_t  c_dir   = 0;                              // 直角方向：-1=左直角, 1=右直角, 0=无直角
    static uint16_t delay_cnt = 0;                            // 延迟直行倒计时

    if (c_state == 0) {                                       // 正常巡线状态，等待直角触发
        if (crnr != 0) {                                      // 检测到直角 → 进入延迟直行阶段
            c_state   = 1;
            c_dir     = crnr;                                 // 记下方向（哪边直角）
            delay_cnt = CORNER_DELAY;
        }
        return 0;                                             // 未处理，调用者自己算差速
    }
    else if (c_state == 1) {                                  // 阶段1: 延迟直行, 车往前压横线
        *l_out = l_in;                                        // 继续直行，不差速
        *r_out = r_in;
        if (--delay_cnt == 0) {                               // 倒计时到0 → 进入转弯
            c_state = 2;
        }
        return 1;                                             // 已处理
    }
    else {                                                    // c_state == 2: 阶段2 原地转弯
        if (c_dir == -1) {                                    /* 左直角: 左轮刹死, 右轮走 — 原地左转 */
            *l_out = 0;
            *r_out = r_in;
			if ( WDNL(L) ) {        				/* 中间两路(ML+MR)扫到黑线 → 转回来了, 退出直角模式 */
				c_state = 0;
				return 2;                                         // 刚退出直角
			}
        } else {                                              /* 右直角: 右轮刹死, 左轮走 — 原地右转 */
            *l_out = l_in;
            *r_out = 0;
			if ( WDNL(R) ) {
				c_state = 0;
				return 2;                                         // 刚退出直角
			}
        }

        return 1;                                             // 还在转弯
    }
}














































































