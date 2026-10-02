# 底盘 + SLAM 项目交接文档

> 本文档记录截至当前的全部架构、已完成工作、最终参数、踩过的坑、剩余计划。
> 下一个接手的人先读这份，再动代码。
> 相关文档：`docs/底盘硬件资源表与编码器标定.md`（编码器标定细节，本文档是其汇总 + 补充）。

---

## 一、项目总览与架构（已冻结）

### 硬件：双 F407 + Jetson Nano

```
                    ┌─────────────────────────────────────┐
                    │           Jetson Nano (上位机)        │
                    │                                     │
   firmware_fc      │  anorosdt2 节点  → /imu/data         │
   (独立飞控F407)───┤  (串口 /dev/ttyUSB0, 921600, ANO协议) │
   (ANO IMU姿态)    │                                     │
                    │  chassis_bridge 节点 → /odom         │
   Telemetry框架    │                  ← /cmd_vel          │
   (底盘F407)───────┤  (串口 USART2 PD5/PD6, 115200)        │
   (电机/编码器/    │                                     │
    里程计)         │  lslidar_driver → /scan              │
                    │  (LSN10 激光雷达)                     │
                    └─────────────────────────────────────┘
```

### 关键决策（不要再改回）

1. **没有 K230，没有 MaixCAM2 中继**。底盘 F407 用 USART2(PD5/PD6) 直接连 Jetson。
2. **firmware_fc 保持原样不动**：它继续用 ANO 协议给 Jetson 发 IMU（`anorosdt2` 节点）。
3. **Telemetry framework 只做底盘**：电机 PWM、编码器、速度环、里程计、底盘串口。
4. **Jetson 端现有代码（chassis_bridge.py 等）一个字节不改**，协议完全兼容。
5. 编码器**集成在 PCB 上，不可改线**，所以必须用软件解码（见下）。

---

## 二、硬件资源表（最终冻结）

| 功能 | 外设/定时器 | 引脚 | 状态 |
|---|---|---|---|
| 左轮电机 PWM | TIM3_CH1 | PB4 | 原有 |
| 右轮电机 PWM | TIM3_CH4 | PB1 | 原有 |
| 左轮电机方向 | GPIO | PE14 / PE15 | 原有 |
| 右轮电机方向 | GPIO | PD8 / PD9 | 原有 |
| **左轮编码器 A 相** | TIM1_CH3 | **PE13** | 软件解码（双边沿捕获） |
| **左轮编码器 B 相** | 普通输入 | **PE11** | 中断里读电平判方向 |
| **右轮编码器 A 相** | TIM2_CH3 | **PA2** | 软件解码（双边沿捕获） |
| **右轮编码器 B 相** | 普通输入 | **PA3** | 中断里读电平判方向 |
| 底盘↔上位机 | USART2 | PD5(TX) / PD6(RX) | 115200 8N1 |
| 舵机×4 | TIM4_CH1..4 | PD12~PD15 | 原有 |
| 底盘 IMU (ICM45686) | SPI2 + TIM6 | — | 原有，自用，不参与 SLAM |
| 调试串口 | USART1 | PA9/PA10 | 原有 |

### 编码器为什么必须软件解码（核心约束）

**STM32 硬件正交编码器模式只能接 CH1/CH2**（`TIMx_SMCR` 的 SMS 位只认 TI1/TI2）。
本板编码器焊在 **CH3**（PE13/PA2），所以 CubeMX 的 Encoder Mode 用不了。

采用的等效方案：
- **A 相所在的 CH3** 配成双边沿输入捕获 → 每个边沿一次中断 → 2 倍频
- **B 相（CH2/CH4 或普通脚）** 当普通输入，中断里读电平判方向
- 判向用 **2 位正交状态机**（比读单点电平抗干扰强）

代价：分辨率减半（2 倍频而非 4 倍频）。对 13 线/1:30 的编码器完全够用。

---

## 三、已完成的工作（全部验证通过）

### 阶段 1：底盘（✅ 完成）

| 项目 | 验证结果 |
|---|---|
| 电机开环控制 | ✅ `Load(-20,-20)` 前进 |
| 编码器读取 | ✅ 静止为 0，无干扰 |
| 编码器方向 | ✅ 前进计数为正（两轮 dir_sign = -1） |
| 编码器标定 | ✅ 推 1m 读 1010mm（误差 +1%），转 1 圈 794 计数（理论 780） |
| 里程计 x/y/θ | ✅ 推 13cm → x=130.7mm，θ≈0 |
| 速度环 PID | ✅ 用户调定 Kp=100, Kd=5, Ki=0 |
| OLED 显示 | ✅ 全整数（MicroLIB 不支持 %f） |

### 阶段 2：串口协议（✅ 完成，USB-TTL 双向验证通过）

| 项目 | 验证结果 |
|---|---|
| 0xCC 发送里程计 | ✅ 50Hz，帧结构/校验/数值全对 |
| 0xBB 接收速度指令 | ✅ 前进/停车/转向都响应 |
| 速度指令超时停车 | ✅ 250ms 无指令自动停 |

### 修过的重要 bug（对应关系见"踩过的坑"）

1. C++ 链接：`extern "C"` 包裹（否则 L6218E undefined symbol）
2. 中断向量：TIM1_CC/TIM2 handler 必须声明（否则覆盖不了 startup 的 WEAK，中断静默死循环）
3. `uint8_t dir_sign = -1` → 255（×255 假计数，这是"扭一下涨10倍"的真凶）
4. 输出钳位 `if(out<0)out=0`（害右轮不转，已删）
5. MicroLIB %f 显示乱码（改 %d ×1000）

---

## 四、关键参数（最终调定值）

### 编码器 / 机械（`Device/Hardware/encoder.h`）

```c
ENC_LINES_PER_REV  = 13.0f    // 编码器线数
ENC_GEAR_RATIO     = 30.0f    // 减速比
ENC_DECODE_FACTOR  = 2.0f     // 2 倍频（软件双边沿）
WHEEL_DIAMETER_M   = 0.065f   // 轮径 65mm
WHEEL_SEPARATION_M = 0.158f   // 轮距（185-27=158，未实测复核，留给 SLAM 阶段）
// 派生: 780 count/圈, ENC_METERS_PER_COUNT = 0.0002618 m/count
```

### 速度环 PID（`Appliaction/Chassis_task.c`，用户调定）

```c
s_pid_left_param  = { 100.0f, 0.0f, 5.0f }   // Kp, Ki, Kd
s_pid_right_param = { 100.0f, 0.0f, 5.0f }
CHASSIS_SPEED_LPF = 0.1f                      // 速度反馈低通
CHASSIS_MAX_WHEEL_MPS = 0.60f                 // 最大轮速限幅
CHASSIS_ENC_LEFT_SIGN  = -1                   // 前进为正的方向修正
CHASSIS_ENC_RIGHT_SIGN = -1
```

> 注意：Ki=0 是纯 PD，有稳态误差（act≈104 而非 200），这是正常的。
> 0.2 m/s 时编码器每 5ms 只有 2 个计数，速度环本身会抖，**正常导航速度(0.3~0.5 m/s)下自然稳**。

---

## 五、通信协议（0xBB / 0xCC）

与 `ros2_ws/src/wheeltec_chassis/wheeltec_chassis/chassis_bridge.py` 完全对应，**Jetson 端不改**。

### 上位机 → F407（速度指令，7 字节）

```
[0] 0xBB            帧头
[1] v_linear 高字节  (int16 大端, mm/s)
[2] v_linear 低字节
[3] v_angular 高字节 (int16 大端, mrad/s)
[4] v_angular 低字节
[5] 校验 = (byte1+byte2+byte3+byte4) & 0xFF
[6] 0x55            帧尾
```

### F407 → 上位机（里程计，15 字节，50Hz）

```
[0]    0xCC          帧头
[1..4] x     (float32 小端, mm)
[5..8] y     (float32 小端, mm)
[9..12]theta (float32 小端, rad)
[13]   校验 = (byte1..byte12 求和) & 0xFF
[14]   0x55          帧尾
```

### 测试指令（USB-TTL / 串口助手十六进制）

```
前进 0.2 m/s :  BB 00 C8 00 00 C8 55
停车         :  BB 00 00 00 00 00 55
原地左转 500 mrad/s : BB 00 00 01 F4 F5 55
```

### 超时停车

`Chassis_protocol.c` 里 `HOST_CMD_TIMEOUT_MS = 250`。超过 250ms 没收指令自动 `Chassis_Stop()`。

---

## 六、代码文件清单

### 新建文件

| 文件 | 内容 |
|---|---|
| `Telemetry framework/Device/Hardware/encoder.c/.h` | 编码器软件解码（CH3 双边沿捕获 + 状态机判向） |
| `Telemetry framework/Appliaction/Chassis_task.c/.h` | 底盘任务：符号归一化 + 差速运动学 + 速度环 + 里程计 + 调试显示 |
| `Telemetry framework/Appliaction/Chassis_protocol.c/.h` | 串口协议：0xBB 解析 + 0xCC 发送 + 超时停车 |

### 修改文件

| 文件 | 改动 |
|---|---|
| `Core/Src/tim.c` | 新增 `MX_TIM1_Init`/`MX_TIM2_Init`（输入捕获）+ `HAL_TIM_IC_MspInit` |
| `Core/Inc/tim.h` | `htim1`/`htim2` 句柄与函数声明 |
| `Core/Src/stm32f4xx_it.c` | 新增 `TIM1_CC_IRQHandler`/`TIM2_IRQHandler` 转发给编码器 |
| `Core/Inc/stm32f4xx_it.h` | 中断原型声明（extern C 块内） |
| `Core/Src/main.c` | 初始化顺序 + USERKEY 调试模式切换 |
| `Core/Inc/main.h` | `#include <stdbool.h>` |
| `Appliaction/Timer_task.c` | 200Hz 调 `Chassis_Update`，50Hz 调 `Host_SendOdom` |
| `boards/bsp.h` | 新增 include（Chassis_task.h / Chassis_protocol.h） |
| `boards/drv_uart.h`、`drv_can.h` | `#include <stdbool.h>`（bool 类型） |
| `MDK-ARM/Telemetry framework.uvprojx` | 加入 3 个新 .c 文件 |

### 已有文档

- `docs/底盘硬件资源表与编码器标定.md`（编码器标定 5 步流程 + 计数疯涨排查）

---

## 七、踩过的坑（重要！接手人必读）

1. **Keil AC5 把 `.c` 按 C++ 编译**（所以有 `MicroLIB does not support C++` 警告）。
   - 后果：函数符号全被 C++ 修饰，跨文件/中断向量对不上 → L6218E undefined symbol。
   - 解法：中断处理函数和跨文件函数都要在头文件里用 `extern "C"` 包裹。
   - **中断处理函数尤其致命**：`TIM1_CC_IRQHandler`/`TIM2_IRQHandler` 没有 C 链接时，覆盖不了 startup 的 `[WEAK]` 弱符号，**链接通过但中断静默跑进死循环，轮子怎么转都不计数**。

2. **`uint8_t dir_sign = -1` 会变成 255**。
   - 后果：编码器增量被 ×255，一次推车计数爆炸（"扭一下涨 10 倍"就是这个）。
   - 解法：`dir_sign` 必须是 `int8_t`。

3. **速度环输出钳位 `if(out<0) out=0` 会把闭环废掉**。
   - 后果：PID 想刹车时输出被强制成 0，电机停转，右轮不转。
   - 解法：不要在输出层砍负值，正确做法是限制 PID 的 `Iout`（抗积分饱和）。

4. **架空测试闭环会炸**：无负载 → 轮子冲高速 → PID 反转 → 电机电流突变 → EMI 串进编码器线 → 假计数（act 几万）。**速度环必须放地上（有负载）测**。

5. **MicroLIB 不支持 %f**：`OLED_printf` 传 `%.3f` 会显示成乱码/原文。解法：全部乘 1000 转整数用 `%d`。

6. **Keil 工程加新文件**：改 `.uvprojx` 磁盘文件后，如果 Keil 开着，会用内存里的旧工程覆盖掉。**最可靠是在 Keil GUI 里右键组 → Add Existing Files**。

7. **编码器接线固定 CH3/CH4**，CubeMX 里能选到定时器通道，但选 Encoder Mode 时引脚会跳走（证明 CH3/CH4 无编码器通路）。

---

## 八、剩余计划（阶段 3 / 4）

### 阶段 3：单独验证 SLAM（未开始）

先不启动 Nav2，只跑：LSN10 激光 + 飞控 IMU + 底盘 odom + Cartographer + RViz。

依次验证：
- `/scan` 正常
- `/imu/data` 正常（anorosdt2 已发布，注意话题名，之前是 `/imu/data` 不是 `/imu/data_raw`）
- `/odom` 正常（阶段 2 已通）
- **TF 树无冲突**：`odom → base_link`（chassis_bridge 出），`base_link → laser/imu`（robot_state_publisher 或 static_transform_publisher 出）
- 直线运动轨迹直不直
- 原地旋转角度准不准（**这是检验轮距 0.158 的时候**，不准就调 `WHEEL_SEPARATION_M`）
- 回环是否正常
- 地图有没有拉伸/重影/旋转漂移

### 阶段 4：接 Nav2（未开始）

地图稳定后：
- AMCL 或 Cartographer localization
- global/local costmap
- DWB 或 Regulated Pure Pursuit
- 速度限制、目标点导航、自动避障、断线停车、导航状态反馈

### 待办/未决事项（按优先级）

1. **轮距 0.158 未实测**：这是算出来的（185-27），原地旋转角度不准时要重新量左右轮触地点中心距。
2. **IMU 安装方向未定**：飞控 IMU 的哪个轴朝车头？只有实测能确定，影响 Cartographer。可以先量芯片朝向 + 板子朝向记下来。
3. **`navigation.launch.py` 需检查**：map server、AMCL 生命周期管理、底盘 TF 与 Cartographer TF 职责划分。
4. **调试代码清理**：`main.c` 的 USERKEY 4 模式切换（模式0标定/1开环/2闭环/3自检）、`Chassis_task.c` 里的 `Chassis_DebugDisplay*`、`Chassis_SelfTest`、`Chassis_DebugDisplayIsr` 都是调试工具，稳定后可精简。
5. **速度环 Ki=0**：纯 PD 有稳态误差，如需要精确速度可加小 Ki（2~5）。

---

## 九、当前状态与下一步

### 当前状态

- 阶段 1（底盘）✅ 完成
- 阶段 2（串口协议）✅ 完成，USB-TTL 双向验证通过
- 阶段 3（SLAM）⏳ 未开始
- 阶段 4（Nav2）⏳ 未开始

### 下一步（接手人从这里开始）

1. **接 Jetson**：F407 `PD5→Jetson RX`，`PD6→Jetson TX`，GND 共地。
2. **找串口号**：Jetson 上 `ls /dev/ttyUSB*`。
3. **配 `chassis_bridge.py` 的 `serial_port`** 指到 F407 的串口。
4. **启动桥接**，验证：
   - `ros2 topic list` 看到 `/odom`、`/cmd_vel`
   - `ros2 topic echo /odom` 推车时 x/y 变化
   - `teleop_twist_keyboard` 键盘控制，**确认方向**（前进=前进，左转=左转；反了在协议层或底盘层修）
5. 方向对了就进阶段 3（SLAM）。

---

## 十、给接手人的一句话总结

底盘已经是一个**能收速度指令、能按指令跑、能回传精确里程计、断线自动停车**的完整底盘了。
剩下的核心风险在 **TF 树**（odom/base_link/laser/imu 的父子关系别搞混）和 **IMU 安装方向**。
编码器、里程计、串口协议这些地基已经打牢，别再动了。
