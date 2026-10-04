# STM32 实时控制固件

[English](firmware.md)

## 当前实现

| 范围 | 当前实现 |
|---|---|
| MCU 基础 | STM32 执行、SWD、USART2、时钟与安全启动 |
| 外设 | ADC、轮编码器、ICM-42688-P 与 TIM1 PWM |
| 实时控制 | FreeRTOS、安全监督、看门狗、TB6612 输出与轮速控制 |
| CAN FD 集成 | FDCAN 上的 Protocol 1.0 与 Orange Pi SocketCAN `can3` |
| 底盘调试 | BODY_COMMAND_READY、Motion Authority、0.30 m/s 闭环直行与遇障停车 |

传输接口为 Orange Pi CAN3 / SocketCAN `can3`；共享线协议为 Protocol 1.0。

## MCU 基线

| 项目 | 配置 |
|---|---|
| Target | STM32G474RET6，LQFP64；DeveBox STM32G474R Ver:20 |
| Clock | 8 MHz HSE，PLL M=2/N=85/R=2，SYSCLK/HCLK/PCLK1/PCLK2 170 MHz |
| Debug / Time Base | 保留 SWD；TIM6 提供 HAL 1 ms；SysTick 提供 FreeRTOS 1 ms Tick |
| PWM | TIM1 CH1/CH2，10 kHz，PSC=0，ARR=16999，启动 CCR=0 |
| Encoder | TIM2/TIM3 Hardware Encoder Mode，均为 0..65535 的 16 位范围 |
| IMU | SPI1 Mode 0，ICM-42688-P 100 Hz 基线，EXTI 唤醒任务 |
| Battery | ADC1 IN6，执行 ADC 校准，软件触发长采样 |
| Diagnostics | USART2 115200 8N1 完整块串行 CLI；默认安静 |
| Watchdog | IWDG 名义约 4 s；仅 Supervisor 刷新；调试暂停时冻结 |
| FDCAN | Nominal 500 kbit/s，Data 2 Mbit/s，FD+BRS，Standard ID |

引脚图和安全启动电平见 [hardware.zh-CN.md](hardware.zh-CN.md)。

## 硬件观测

集成板级测试得到以下结果：

- STM32 执行与 USART2 实际字节；
- 真实编码器采集；
- ICM-42688-P 通信与真实惯性数据；
- Battery ADC 测量链路；
- TIM1 PWM 输出；
- TB6612 电机控制电气链路及真实电机转动。

编码器符号为：

```text
左轮正转  -> raw CPS < 0
右轮正转  -> raw CPS > 0
逻辑正转  -> 两侧归一化速度均 > 0
```

符号修正位于 `app_drivetrain`，不修改 Timer Encoder Driver。Commissioning 确定 Encoder 1
属于右轮且正向 Raw Sign 为 `+1`，Encoder 2 属于左轮且正向 Raw Sign 为 `-1`。Encoder 1/2 Raw
诊断保持不变；逻辑轮状态和 Protocol `0x181` 使用右轮=`+raw`、左轮=`-raw`。调试确定
Motor A 属于右轮、Motor B 属于左轮，两路电机的逻辑正向符号均为 `-1`；归一化仍位于 Motor
GPIO/PWM Driver 上层。

## 生产源码架构

CubeMX 管理的初始化位于 `firmware/Core`，项目自有逻辑位于 `firmware/App`。

| 模块 | 职责 |
|---|---|
| `app_config` | 固件身份、调度、安全和通信常量 |
| `app_state` / `app_safety` | BOOT/INIT/SAFE/READY/ACTIVE/FAULT 与集中安全收敛 |
| `app_supervisor` | 关键任务心跳和唯一 Watchdog Feed Owner |
| `app_command` | 共用命令、来源、时间戳与本地 Freshness |
| `app_drivetrain` | 轮/电机映射、逻辑符号、标定 Guard、差速换算 |
| `app_control` | 左右轮独立 PI/PID-compatible 控制、限幅与 Anti-windup |
| `app_motor` | TB6612 Direction/PWM/STBY 与 Emergency-safe 输出 |
| `app_encoder` | 16 位 Wrap-safe Delta、累计计数、Raw/Filtered CPS |
| `app_imu` | ICM Register Driver、WHO_AM_I、数据采集与中断状态 |
| `app_battery` | ADC Calibration、分压模型、滤波估计与标定有效性 |
| `app_telemetry` | 一致的 Sensor/Control/Safety Snapshot |
| `app_protocol` | Protocol v1 显式 Little-endian 编解码；禁止 Packed Struct Cast |
| `app_can` | FDCAN Filter、ISR RX Ring、Session/Sequence/运行限值校验、CAN Health、TX Schedule |
| `app_diagnostics` | 轻量非阻塞完整块串行 UART 控制台、1 Hz Watch 与链路诊断 |
| `app_rtos` | 紧凑任务模型与确定性所有权 |

已部署固件版本为 `0.5.5`，共享线协议版本保持 `1.0`。
固件 0.5.5 已通过生产导航验收：桥接通信恢复后，2026-09-08 的实车去程及返程均成功，
未出现 COMMAND_TIMEOUT；2026-09-13 全图及薄墙导航通过。见 [CHANGELOG](../CHANGELOG.md)。

## FreeRTOS 与中断架构

| Task | Priority | Stack | 基线行为 |
|---|---:|---:|---|
| SupervisorTask | Realtime | 2048 B | 20 ms Safety / Watchdog Cycle |
| MotorControl | High | 1024 B | 10 ms Encoder / Control / Output Path |
| Imu | AboveNormal | 1024 B | EXTI Wake，20 ms Fallback |
| Communication | Normal | 2048 B | Event Wake 或最大 5 ms Wait；UART + FDCAN Task-context 处理 |
| Telemetry | BelowNormal | 1024 B | 50 ms Service；只负责 Battery Acquisition 调度 |

FDCAN1_IT0 Priority 为 5，与 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 相容。ISR 将三项
Hardware FIFO 搬运到有界八槽项目 Ring，记录 Overflow/Error，并唤醒 CommunicationTask；
ISR 不解析 Protocol v1，也不改变电机目标。协议校验与 Telemetry Serialization 均位于任务上下文。

## 安全与 CAN Authority

生产状态机保持：

```text
BOOT -> INIT -> SAFE -> READY -> ACTIVE
                     \          /
                      -> FAULT <-
```

上电和所有 Fault Path 均强制 STBY LOW、Direction LOW、PWM 0。CAN 不能绕过既有 Supervisor、
内部 Command Model、Drivetrain Guard、Wheel Controller 或 Motor Layer。

执行器授权合同为：

```text
显式主机意图（导航目标或调试 START）
-> Motion Authority Granted
-> Fresh Motion Commands
-> Motion Allowed
```

系统默认禁止运动，Motion Authority 独立于 Velocity Command。停止过程采用分层撤销 Motion
Authority：

```text
Authority Withdrawal / Stale Command / Communication Loss / Fault / Explicit STOP
-> 在可用处发送 Zero Motion Command
-> Motion Authority Withdrawn
-> STM32 Motor-safe Stop
```

Motion Authority 丢失或被撤销都会停止运动。Command Freshness、Heartbeat/Watchdog 与 Fault
Handling 仍是相互独立的底层保护。独立遇障停车演示采用锁存 STOPPED，并要求再次显式
START。生产 Nav2 自行处理障碍和恢复；主机运动门控保留显式目标授权及 STOP／故障锁存。
见[导航操作](rviz_vm.zh-CN.md)。

每个 CAN `BODY_VELOCITY` 写入共享命令模型之前，都会检查 Controller Configuration、Drivetrain
Readiness 和本地 Operating Limits。配置不完整或越界的 Active Command 会被拒绝，并通过既有
Safety Supervisor 安全收敛；绝不 Clamp 成另一种运动。

Protocol v1 的激活顺序是：

```text
Fresh Heartbeat
-> 显式 DISARMED Handshake
-> 显式 ARMED Authority
-> Fresh BODY_VELOCITY Command
-> 所有 Drivetrain / Controller / Safety Gate
-> ACTIVE
```

Motion Command Timeout 为 250 ms，Host Heartbeat 与 Motion Authority Timeout 均为 500 ms，
全部使用 STM32 本地接收时间。Timeout、Session Replacement、Bus Off 或 RX Loss 都会清除 CAN Authority，绝不会
复用旧速度。恢复必须重新执行 DISARMED/ARMED。

### 0.5.5 命令时效修复：已验收

已在生产源码与确定性主机复现中 VERIFIED：CommunicationTask 在处理 RX 队列前
获取 `now_ms`，而 BODY_VELOCITY 接受路径在 `AppCommand_Submit` 中使用较晚的
`HAL_GetTick()`。如果期间发生毫秒跳变，`1000 - 1001` 的无符号运算得到
4294967295 ms，`AppCan_ProcessHealth` 因而可能对刚接受的命令锁存 COMMAND_TIMEOUT。
两者均使用 TIM6 HAL 时钟，并非 HAL/RTOS 混用；任务抢占可扩大窗口，现有 IRQ Lock
已防止命令结构撕裂。历史 CAN 日志不能定位故障当时的具体中断指令。

修复在 RX 队列处理完成后重新采样 CAN 检查时间，并在现有命令快照 IRQ Lock 内
采样 HAL 时间。保留无符号回绕计算、原有 `> 250 ms` 判断、500 ms Authority/Heartbeat、
故障锁存和显式恢复；不修改协议、调度优先级或执行器逻辑。修复已随 0.5.5 部署，
并通过后续实车导航验收。

最小验证：`firmware/tests/command_freshness_test.c` 直接运行生产 Command/Safety C，
仅替换时钟和执行器依赖。在 Orange Pi 主机编译器上，原代码的新鲜命令断言失败，
修复后通过旧调用时间、250 ms 有效/251 ms 故障、时钟回绕、安全输出请求及恢复流量
不自动清除故障验证。这不是 STM32/CAN 实机验收。Debug 构建成功：FLASH 105552 B，
RAM 42848 B，产物位于 `firmware/build/Debug/firmware.{elf,hex,bin}`。
日志窗口确认命令 789 于 1788785844.424471 发送，1788785844.454335 的状态帧报告
故障 0x00000002、已接受序号 789。

2026-09-08 部署：通过现有 J-Link SWD 路径（STM32G474RE、4000 kHz、探针
602710753）完成烧录、校验、复位与运行。HEX SHA256：
`9A7EC1FC58AAC2C70381FA7EB64F26DF18A7C639A7CF468F0D2DF83149C7BE91`。
只读 SWD 回读确认已部署版本字符串 0.5.5、初始化完成、READY、fault_flags=0、
电机未授权、STBY LOW、TIM1 两路 PWM Compare 为零；烧录前也确认 STBY LOW/PWM 零。
未发送运动命令。

2026-09-08 初次部署曾被已有 CAN 通信故障暂时阻塞。当日稍后重启受抑制的桥接连接后，
通信恢复，实时状态确认 0.5.5、传输健康且无故障。随后两个实车导航目标均成功，
没有 COMMAND_TIMEOUT 或健康故障停车，桥接错误／拒绝计数为零，到达后撤权。
部署阻塞已关闭，但这不代表已查明原通信故障的物理原因。Protocol 1.0 保持 FROZEN。

Disabled/Disarmed Command Snapshot 被明确视为未超时。只有已经建立的 Motion/Control Session
失去必要 Freshness（例如 Valid Motion Command 到期，或 Armed CAN Authority/Heartbeat 超时）
时，`COMMAND_TIMEOUT` 才生效；Boot/SAFE 状态不会仅因没有命令而产生该 Fault。

Encoder Acquisition Validity 与 Drivetrain/Controller Readiness 相互独立。Timer Start 失败、
Invalid Sample 或任一路 Sample Age 超过 50 ms 时置位 `ENCODER_VALIDITY`。只有两路 Encoder
Timer 均已初始化成功、两路 Sample 均恢复有效且 Age 不超过 50 ms 时，该位才会自动清除。
未知 Motor A/B Mapping/Polarity、未配置 Gain/Limit 和 `BODY_COMMAND_READY=false` 都不会置位它。

当通信丢失影响 CAN 所有的运动时，集中 Fault Word 设置 FDCAN Communication Bit。Warning
与 Error Passive 保持可观测，不建立第二套 Safety State Machine。固件每 1 s 尝试一次 Bus-off
恢复，但硬件恢复不会恢复运动授权。

USART2 以紧凑格式显示 System/Sensor/Fault，以及 RX/TX/Reject/Sequence/Overflow/Bus-off Count、
STM32 本地 Heartbeat/Authority/Command Age、Session ID、Authority 与 Motor Enable。启动块后没有
非请求周期遥测；Watch 只支持 Encoder、IMU、CAN，全部固定为 1 Hz。完整响应块串行输出，UART
Transmission 不阻塞 MotorControlTask。全部命令见
[STM32 USART2 生产版 CLI](uart_cli.zh-CN.md)。

## Protocol v1 Transport

| 项目 | 数值 |
|---|---|
| Nominal Timing | Prescaler 17，SEG1 15，SEG2 4，SJW 4；500 kbit/s，80% Sample Point |
| Data Timing | Prescaler 5，SEG1 13，SEG2 3，SJW 3；2 Mbit/s，82.3529% Sample Point |
| RX IDs | `0x080` Authority、`0x081` Motion、`0x082` Heartbeat |
| TX IDs | `0x180` System、`0x181` Wheel、`0x182` IMU、`0x183` Battery |
| Integrity | 仅 CAN FD Link CRC；无 Application CRC |
| RX Policy | 精确 Filter；校验 FD+BRS、Length、Version、Reserved、Session、Sequence |
| TX Policy | CommunicationTask 唯一负责确定性调度；其他模块只更新内部 Telemetry |

完整字段偏移、Golden Frame、ROS 换算和 Fault 含义见
[Protocol v1](../interfaces/protocol_v1.md)。

## Drivetrain 配置边界

集中固件默认值现已包含用户实测的 Commissioning Geometry：

| 数值 | Commissioning Setting | 依据 |
|---|---:|---|
| Wheel Radius | 0.023 m | `MEASURED / COMMISSIONING` |
| Wheel Track | 0.125 m | `MEASURED / COMMISSIONING` |
| 差速换算使用的 Half Track | 0.0625 m | `DERIVED` |
| Geometric Circumference | 0.1445132621 m | `DERIVED` |
| Encoder 1 归属 / 正向符号 | 右轮 / `+1` | 实机方向测试 |
| Encoder 2 归属 / 正向符号 | 左轮 / `-1` | 实机方向测试 |
| Motor A 归属 / 逻辑正向符号 | 右轮 / `-1` | 实机方向测试 |
| Motor B 归属 / 逻辑正向符号 | 左轮 / `-1` | 实机方向测试 |
| 右轮每转计数 | 1059.5 | `MEASURED / COMMISSIONING` |
| 左轮每转计数 | 1060.8 | `MEASURED / COMMISSIONING` |
| 右轮每 Count 米数 / 弧度 | 0.0001363976 m / 0.005930331 rad | `DERIVED` |
| 左轮每 Count 米数 / 弧度 | 0.0001362305 m / 0.005923063 rad | `DERIVED` |
| 最大车体线速度 | 0.30 m/s | `COMMISSIONING LIMIT` |
| 最大车体角速度 | 1.50 rad/s | `COMMISSIONING LIMIT` |
| 最大归一化轮速 | 3000 count/s（约 0.409 m/s） | `COMMISSIONING LIMIT` |
| 轮目标斜率 | 4400 count/s^2（约 0.60 m/s^2） | `COMMISSIONING LIMIT` |
| 电机 Effort / PWM 输出 | +/-0.60 normalized | `COMMISSIONING LIMIT` |

固件实现的 Differential-drive Relationship 因此为：

```text
v_left  = v - omega * 0.0625
v_right = v + omega * 0.0625
```

Encoder Scale 来自每个物理车轮正向完整旋转 10 圈：Encoder 1 累计 +10,595 counts，Encoder 2
累计 -10,608 counts。两侧约 0.123% 的差异会分别保留，固件不取平均。Radius、Track 和 Encoder
Scale 是 Commissioning Value；有效 Odometry Calibration 仍需测量真实直线行驶距离与旋转运动。

初始左右独立 PI 配置均为 `Kp=0.00020`、`Ki=0.00060`、`Kd=0`，Integrator Limit 为
700 count-seconds，Output Limit 为 0.60。这是有意相同的初始调试参数，不是已经实车调谐的 Gain。
控制器会拒绝非有限值、执行 Output Clamp 和 Conditional-integration Anti-windup，并只在有效
Active Command 下使用目标斜坡。任何 Safety Convergence 都会复位 Controller 并直接强制 Motor
Safe，Authority Withdrawal 不会被斜坡延迟。

初始化时，只有已配置 Mapping/Scale/Geometry、有限 Operating Envelope 和两个有效 Controller
Configuration 同时存在，BODY_COMMAND_READY 才为 true。Protocol `0x180` bit 15、`0x181` bit 12、
UART `status` 和 Safety Guard 都采用这一完整条件。越界 BODY_VELOCITY 会被拒绝，不会被 Clamp。

0.30 m/s 是固件调试上限，不是实测的电机或整机最高速度。实际安装电机的负载/堵转电流、
带载可达轮速以及 TB6612 Carrier/Module 热裕量尚未实测，因此目前无法为更高持续速度提供安全
依据。0.60 Output Clamp 只限制 Duty Request；由于当前固件链路没有电流反馈，它不等于电流限制。
Encoder PPR/CPR 定义与 Gear Ratio 分解仍未知，但直接输出轮 Count Scale 已使它们不再是 Body
Conversion 的必要条件。禁止从 Nav2 Footprint 推导底盘几何。

## Telemetry 与 ROS 边界

固件输出归一化累计轮计数/速度、保持不变的 Raw Encoder 1/2 Diagnostic Field、Controller
Target/Output、IMU SI 数据、Battery Estimate/Validity、Safety State、Fault Word、
Supervisor/Watchdog Health、STM32 Monotonic Time 与 Sequence。Raw 值也继续通过本地 UART 提供；
它们与 Protocol `0x181` 的逻辑字段并存且不会替代逻辑字段。固件不生成 ROS Odometry、ROS Time、
Orientation、Covariance 或 TF。

Orange Pi 拥有 `nav_msgs/msg/Odometry`，并在后续负责唯一 `odom -> base_link` TF Authority。
只有完成底盘尺度实测与真实运动一致性验收后，才能接受公制 Odometry。

## 构建与硬件证据

2026-08-29 使用 Arm GNU Toolchain 完成 Debug 与 Release Clean Build，Compiler/Linker Warning
均为 0，并生成 ELF、HEX、BIN：

| Build | FLASH | RAM | Warning |
|---|---:|---:|---|
| Debug | 105,544 B / 512 KiB (20.13%) | 42,848 B / 128 KiB (32.69%) | 0 |
| Release | 67,936 B / 512 KiB (12.96%) | 42,848 B / 128 KiB (32.69%) | 0 |

跨域物理链路使用 Orange Pi CAN3 / SocketCAN `can3`：在 Nominal 500 kbit/s、Data
2 Mbit/s 下真实观察到 FD+BRS `0x080`、`0x082` 和 `0x180..0x183`。Linux 显式使用 80% / 82.5%
Sample Point 后保持 Error Active，TX/RX Error 与 Bus-off 均为 0。Protocol 1.0、STM32 Timing
和机器人默认 DISARMED 行为均未改变。

遇障停车演示使用固件 `0.5.4`：BODY_COMMAND_READY 与 Motion Authority 均存在，
系统以当前 0.30 m/s Commissioning Limit 进行闭环直行；真实 RPLIDAR A1 `/scan` 在 30° 前方
扇区和 0.60 m 阈值内检出障碍物后，触发 Zero Velocity、Authority Withdrawal 与 STM32 Safe
Stop。障碍物移除后 STOPPED 仍锁存，必须由用户重新显式 START 才能恢复运动。该调试速度不代表
机器人物理最高速度。

从 Obstacle Detection 起，观测到 Zero Velocity Command 约 0.075 ms、Motion Authority
Withdrawal 约 1.276 ms、STM32 Stop Confirmation 约 30.8 ms。这些是成功 Demo 的观测值，不是
有保证的最坏情况安全上限。
