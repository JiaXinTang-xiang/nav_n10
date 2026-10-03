# PC 端交接文档 — ROS2 多机通信（WiFi 遥控 + 显示）

> **给谁看**：PC 端的 AI / 操作者。
> **目的**：Jetson（小车上的主控）已经配好多机通信，PC 通过 WiFi 遥控 + 显示。本文说明 Jetson 端做了什么、PC 端要做什么、怎么连接调试启动。
> 建图本身的命令见 [RUNBOOK.md](RUNBOOK.md)，本文只讲「两台机器怎么连起来」。

---

## 1. 网络拓扑与分工

```
                    WiFi 局域网（同一个 AP，关 AP 隔离）
   ┌─────────────────────────────────────────────────┐
   │                                                 │
   │  Jetson（车上，主脑）          PC（遥控 + 显示）   │
   │  IP: 192.168.166.76 (DHCP)     IP: 192.168.166.45 │
   │  user: jiaxintang              user: <PC 用户>   │
   │                                                 │
   │  跑：三路传感器 + RSP +       跑：RViz + 键盘遥控  │
   │      Cartographer + 地图        （SSH 进 Jetson 发指令）
   └─────────────────────────────────────────────────┘
```

**分工原则**：所有「干活」的节点都跑在 Jetson 本地（传感器→SLAM 闭环在车上），PC 只做两件事——**SSH 进去发指令** + **本地跑 RViz 显示 / 键盘遥控**。这样 PC 断网也不影响建图。

---

## 2. Jetson 端已做的配置（PC 端 AI 不要再重复改）

在 `~/.bashrc` 末尾追加了以下内容（Jetson 上已生效，交互式 shell 里 `ROS_DOMAIN_ID=0` 已验证）：

```bash
# ================= ROS2 多机通信（Jetson 端）=================
export ROS_DOMAIN_ID=0          # 与 PC 保持一致（两边都必须是 0）
export ROS_LOCALHOST_ONLY=0     # 允许跨机通信，绝不能设成 1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jiaxintang/Desktop/nav_n10/cyclonedds/cyclonedds-jetson.xml
```

Jetson 端现状快照（PC 端 AI 可据此判断，勿重复配置）：

| 项 | 值 |
|---|---|
| ROS2 发行版 | Humble |
| 用户 / 主机名 | `jiaxintang` / `jiaxintang` |
| WiFi 网卡 | `wlP1p1s0` |
| 当前 IP | `192.168.166.76`（**DHCP，会变**） |
| RMW | CycloneDDS |
| SSH | `sshd` 已运行（`active`） |
| CycloneDDS | 已安装并验证 PC 能收到 Jetson 话题 |
| 防火墙 ufw | 已安装，状态未确认（需 root，见第 6 节） |
| `ROS_LOCALHOST_ONLY` | `0`（正确，绝不要改成 1） |

---

## 3. PC 端要做的配置

### 3.1 前提
PC 必须装了 **ROS2 Humble**（与 Jetson 一致）。没装先装。

### 3.2 连同一个 WiFi + 关 AP 隔离
1. PC 和 Jetson 连**同一个** WiFi。
2. 进路由器后台关掉「**AP 隔离 / 客户端隔离 / Client Isolation / 无线隔离**」——这是多机发现失败的第一大原因。
3. 记下 PC 自己的 IP：`hostname -I`
4. 互 ping 通：`ping 192.168.166.76`（Jetson 的 IP 若变了，先在 Jetson 上 `hostname -I` 重新查）

### 3.3 写 ROS2 多机环境变量（PC 端）
PC 的 `~/.bashrc` 末尾加**同样**三行（关键：`ROS_DOMAIN_ID` 两边必须一致 = 0，`ROS_LOCALHOST_ONLY` 必须 0）：

```bash
# ================= ROS2 多机通信（PC 端）=================
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jiaxintang/桌面/nav_n10/cyclonedds/cyclonedds-pc.xml
```

写完后 `source ~/.bashrc`，或重开终端。

### 3.4 关防火墙（PC 端）
```bash
sudo ufw disable        # 若 PC 上 ufw 是 active 的话；inactive 就跳过
```

---

## 4. 验证连接（先测通，再上全流程）

1. **Jetson 上**（SSH 进去）起数据：
   ```bash
   bash ~/Desktop/nav_n10/slam_bringup.sh
   ```
2. **PC 上**新终端：
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```
   **能看到 `/scan` `/odom` `/imu/data` 等 = 发现成功。**
3. 再抽一帧确认数据真跨机过来了：
   ```bash
   ros2 topic echo /scan --once
   ```

> 若第 2 步看不到话题，跳到第 6 节排查表。

---

## 5. 日常启动流程（谁在哪个终端跑什么）

### SSH 进 Jetson（PC 上执行）
```bash
ssh jiaxintang@192.168.166.76
```

### Jetson 终端 — 一键起三路数据 + SLAM
```bash
bash ~/Desktop/nav_n10/slam_bringup.sh
```

### PC 终端 A — RViz（在 PC 本地跑，不是 Jetson）
```bash
cd ~/桌面/nav_n10
./pc_rviz.sh
# 手动：Fixed Frame = map，Add → /scan (LaserScan)、/map (Map)
```
> 不要用 Jetson 上那份 `lsn10_cartographer.rviz`（它引用了 Jetson 没有的 cartographer_rviz / wyca 插件，会报一堆加载失败但无害）。PC 直接开干净 rviz 手动加显示即可。

### PC 终端 B — 键盘遥控（发 /cmd_vel 到车上）
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 6. 排障 + 需要 sudo 的遗留项

### 排障表

| 现象 | 排查顺序 |
|---|---|
| PC `ros2 topic list` 看不到 Jetson 话题 | ① 同一个 WiFi？② AP 隔离关了没？③ 两边 `ROS_DOMAIN_ID` 都是 0？④ 两边 `ROS_LOCALHOST_ONLY` 都不是 1？⑤ 两边 ufw 关了没？⑥ 互 ping 通不通？ |
| 能看到 topic 但 `echo` 收不到数据 | 组播发现到了、数据单播被挡 → 先关两边 ufw；还不行换 CycloneDDS（见下） |
| 数据卡顿 / 延迟高 | 2.4G 拥挤 → 换 5G，或 PC 用网线接路由器 |
| Jetson IP 变了连不上 | DHCP 会变，Jetson 上 `hostname -I` 重新查；想一劳永逸就固定 IP（路由器里给 MAC 绑定，或 Jetson `nmcli` 设静态 IP） |
| PC 缺 `cartographer_ros_msgs` | 不影响，PC 只跑 RViz 看标准话题（/scan /map /tf），不需要 cartographer 的包 |

### 需要 sudo 的遗留项（Jetson 和 PC 的 sudo 都要密码，跑这些要先问用户要密码）

1. **确认并关闭防火墙**（两边）：
   ```bash
   sudo ufw status        # 先看是不是 active
   sudo ufw disable       # active 才需要关
   ```
2. **换 CycloneDDS（备选，FastDDS 发现失败时才做）**——两边都要装 + 启用：
   ```bash
   sudo apt install -y ros-humble-rmw-cyclonedds
   # 然后两边 ~/.bashrc 里取消注释那一行：
   # export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   # source ~/.bashrc 后重启相关节点
   ```

---

## 7. 给 PC 端 AI 的一句话结论

Jetson 跑传感器与 SLAM，PC 通过 `ssh jiaxintang@192.168.166.76` 远程控制，并在本地运行 `./pc_rviz.sh` 显示。已用 CycloneDDS 实测 PC 能接收 Jetson 发布的 ROS 2 话题。
