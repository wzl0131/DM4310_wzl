# dm4310_driver

达妙 DM4310 电机 ROS 2 驱动，通过 DM-LinkX-4C 四通道 USB-CAN FD 模块控制。

MIT 阻抗控制模式，支持位置 / 速度 / 力矩 / 阻抗四种控制方式，单电机场景。

---

## 特性

| 项 | 说明 |
|---|---|
| 通信 | CAN FD，仲裁段 1 Mbps + 数据段 5 Mbps（BRS=1） |
| 控制模式 | MIT 阻抗控制（τ = kp·Δp + kd·Δv + t_ff） |
| 控制频率 | 1 kHz（与电机反馈同频） |
| 指令接口 | 单话题 `/dm4310/mit_command`，一个消息装 MIT 五参数 + mode |
| 反馈接口 | 标准 `sensor_msgs/msg/JointState` |
| 安全机制 | 首帧同步、刚度恢复对齐、反馈超时切阻尼 |

---

## 环境要求

```
Ubuntu 24.04
ROS 2 Jazzy
DM-LinkX-4C 模块（固件 ≥ v1.1.0.0，旧固件 FD 功能不可用）
达妙 DM4310 电机 + 24V 电源
```

**依赖：**

```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-std-msgs ros-jazzy-sensor_msgs \
                 ros-jazzy-rosidl-default-generators libusb-1.0-0-dev
```

---

## 安装与编译

```bash
mkdir -p ~/dm4310_ws2/src
cd ~/dm4310_ws2/src
# 将本包放到这里
cd ~/dm4310_ws2

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

**`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 用于 VS Code IntelliSense，建议保留。**

---

## USB 权限

不想每次用 sudo，配置 udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="4543", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-dm-linkx.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

重新插拔模块后生效。

> **不要用 `sudo ros2 launch`** —— sudo 会丢失 ROS 环境变量，导致 `ros2: 未找到命令`。

---

## 配置

参数文件：`config/dm4310_params.yaml`

### 电机与总线

| 参数 | 默认 | 说明 |
|---|---|---|
| `can_id` | 12 | 电机接收指令的帧 ID（**必须实测确认，不要用出厂默认 1**） |
| `mst_id` | 44 | 电机反馈帧的 ID |
| `channel` | 0 | DM-LinkX-4C 通道 0~3 |
| `control_rate_hz` | 1000.0 | 控制循环频率 |

### CAN FD

| 参数 | 默认 | 说明 |
|---|---|---|
| `canfd_channel` | true | 通道是否工作在 FD 模式。FD 控制器向下兼容经典帧 |
| `canfd_frame` | true | 发出去的帧是否用 FD 格式（FDF=1） |
| `canfd_brs` | true | **是否启用 BRS**。true = 数据段切到 `canfd_baudrate` |
| `can_baudrate` | 1000000 | 仲裁段波特率（达妙固定 1 Mbps） |
| `canfd_baudrate` | 5000000 | 数据段波特率（仅 `canfd_brs: true` 时生效） |
| `can_sp` | 0.75 | 仲裁段采样点 |
| `canfd_sp` | 0.75 | 数据段采样点（**不能为 0**，否则位时序非法） |

> **`canfd_channel` 和 `canfd_frame` 是两个不同层级的概念：**
> 前者是"通道控制器是否工作在 FD 模式"，后者是"这一帧用哪种格式"。
> 详见 [CAN FD 说明](#can-fd-说明)。

### MIT 增益与安全

| 参数 | 默认 | 说明 |
|---|---|---|
| `kp` | 10.0 | 刚度。消息里 `kp` 填 0 时用此值 |
| `kd` | 0.5 | 阻尼。消息里 `kd` 填 0 时用此值 |
| `timeout_ms` | 100 | 超过此时长无反馈 → 自动切阻尼模式（kp=0） |

---

## 使用方法

### 启动

```bash
source /opt/ros/jazzy/setup.bash
source ~/dm4310_ws2/install/setup.bash
ros2 launch dm4310_driver dm4310.launch.py
```

**正常启动日志：**

```
[INFO] LINKX4C 设备数 = 1
[INFO] Device Version: linkx_4c_app v1.1.0.0
[INFO] 通道 0 就绪，FD 模式  仲裁段 1000000 bps (sp 0.75)，数据段 5000000 bps (sp 0.75)
[INFO] CAN 帧格式：CAN FD 帧（FDF=1），BRS=1（数据段提速）
[INFO] dm4310_node 启动：CAN_ID=12 MST_ID=44 ch=0 1000Hz kp=10.0 kd=0.50
[INFO] 首帧反馈：pos=-0.3119 rad，目标已同步到当前位置
[INFO] 目标 -0.312 | 当前 -0.312 rad | 速度 5.07 | 力矩 0.13 | 温度 22/22
```

**临时覆盖参数：**

```bash
ros2 launch dm4310_driver dm4310.launch.py kp:=5.0 kd:=1.0
ros2 launch dm4310_driver dm4310.launch.py canfd_brs:=false
```

启动日志会打印 `⭐ 命令行覆盖 yaml：{...}` 确认覆盖生效。

### 指令消息

话题：`/dm4310/mit_command`
类型：`dm4310_driver/msg/DmMitCommand`

```
uint8 MODE_POSITION  = 0
uint8 MODE_VELOCITY  = 1
uint8 MODE_TORQUE    = 2
uint8 MODE_IMPEDANCE = 3

uint8  mode
float64 position    # rad
float64 velocity    # rad/s
float64 kp          # 0 = 使用参数默认值
float64 kd          # 0 = 使用参数默认值
float64 torque      # N·m
```

**节点按 `mode` 自动处理 kp/kd，无需手动调参：**

| mode | 名称 | kp | kd | 生效的指令量 |
|---|---|---|---|---|
| 0 | 位置 | 消息值 / 参数默认 | 消息值 / 参数默认 | `position` |
| 1 | 速度 | **强制 0** | 消息值 / 参数默认 | `velocity` |
| 2 | 力矩 | **强制 0** | 消息值 / 参数默认，**最小 0.3** | `torque` |
| 3 | 阻抗 | 消息值 / 参数默认 | 消息值 / 参数默认 | `position` + `torque` |

### 常见控制操作

```bash
# 位置：走到 0.3 rad
ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand \
  "{mode: 0, position: 0.3, kp: 10.0, kd: 0.5}"

# 速度：1.0 rad/s（kp 自动为 0）
ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand \
  "{mode: 1, velocity: 1.0, kd: 0.5}"

# 力矩：0.3 N·m（kd 最小 0.3，防飞车）
ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand \
  "{mode: 2, torque: 0.3, kd: 0.3}"

# 阻抗：位置 0.2 rad + 前馈 0.5 N·m
ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand \
  "{mode: 3, position: 0.2, torque: 0.5, kp: 10.0, kd: 0.5}"

# 停止（回位置模式并归零）
ros2 topic pub --once /dm4310/mit_command dm4310_driver/msg/DmMitCommand \
  "{mode: 0, position: 0.0, kp: 10.0, kd: 0.5}"
```

> 消息中未写的字段自动为 0。因此 `"{mode: 1, velocity: 1.0}"` 等价于完整的五参数形式。

### 查看反馈

```bash
ros2 topic echo /joint_states --once                   # 一帧完整数据
ros2 topic echo /joint_states --field position --once  # 只看位置
ros2 topic echo /joint_states --field velocity --once
ros2 topic echo /joint_states --field effort --once
ros2 topic hz /joint_states                            # 应为 1000 Hz
```

### 运行时调参

```bash
ros2 param get  /dm4310_node kp
ros2 param set  /dm4310_node kp 20.0
ros2 param set  /dm4310_node kd 1.0
ros2 param dump /dm4310_node          # 导出当前参数
```

**可运行时修改：** `kp`、`kd`、`timeout_ms`
**需重启节点：** `can_id`、`mst_id`、`channel`、`control_rate_hz`、`canfd_*`

### 记录数据

```bash
ros2 bag record /joint_states -o ~/dm4310_test
ros2 bag info ~/dm4310_test
```

---

## 代码结构

四层结构，自底向上：

```
src/dm4310_driver/
├── msg/DmMitCommand.msg      指令消息（MIT 五参数 + mode）
├── include/
│   ├── can/can.hpp           CAN 总线封装接口
│   └── motor/
│       ├── motor.hpp         抽象层：Motor / Encoder / Command
│       └── DM_motor.hpp      达妙实现声明
├── src/
│   ├── can/can.cpp           CAN 总线封装实现
│   ├── motor/motor.cpp       型号参数表（PMAX/VMAX/TMAX/KD_MAX）
│   ├── motor/DM_motor.cpp    MIT 打包 / 反馈解析 / 使能失能
│   ├── dm4310_node.cpp       ROS 节点（话题 / 定时器 / 安全逻辑）
│   ├── dm_scan.cpp           工具：扫描 CAN_ID / MST_ID
│   └── sdk_probe.cpp         工具：验证 USB + SDK
├── config/dm4310_params.yaml 参数
├── launch/dm4310.launch.py   启动文件
└── third_party/DMLink/       官方 SDK（勿修改）
    ├── dmcan.h
    └── libdm_device.so
```

### 分层边界

| 层 | 职责 | 不应出现 |
|---|---|---|
| `can` | 字节进出总线 | 电机相关概念 |
| `motor` | 电机抽象状态与操作 | `0xFF`、`MIT`、`dmcan_` |
| `DM_motor` | 达妙协议细节 | `rclcpp` |
| `dm4310_node` | ROS 接口与控制循环 | 直接调用 `dmcan_*` |

**自检：**

```bash
grep -r "rclcpp"  include/motor/    # 应为空
grep -r "0xFF"    include/motor/    # 应为空
```

### 数据流

**下行（指令，1 kHz）：**

```
ROS 话题 /dm4310/mit_command
  → 订阅回调（存 5 个字段 + mode）
  → Loop() 按 mode 计算有效 kp/kd
  → 填 Motor::command
  → DMMotor::MotorControl()
      → CommandPack()：5 个物理量 → 8 字节
      → SendCanMsg() → CanSendMsg() → dmcan_device_send_can()（SDK）
  → USB → LinkX-4C → CAN FD 帧 → 电机
```

**上行（反馈，1 kHz）：**

```
电机每 1 ms 主动回一帧（CAN ID = MST_ID）
  → SDK 接收线程（libdm_device.so 内部）
  → On_Can_Recv()（全局回调，因 SDK 只接受普通函数指针）
      → 校验 dlc、按 MST_ID 过滤
      → 转成 can_frame
  → DMMotor::CanMsgParse()
      → StampFeedbackTime()（atomic 时间戳）
      → 拆位得到 pos_int / vel_int / tor_int
      → IntToFloat() 换算成物理量
      → lock(encoder.mutex) 写入 Encoder
  → Loop() 加锁读出 → 发布 /joint_states
```

### 线程模型

| 线程 | 职责 |
|---|---|
| 主线程（`rclcpp::spin`） | 定时器 `Loop()` + 订阅回调，串行执行 |
| SDK 接收线程 | 收 CAN 帧 → `On_Can_Recv()` → `CanMsgParse()` |

**跨线程数据加锁：**

| 数据 | 保护方式 |
|---|---|
| `Encoder`（位置/速度/力矩/温度/错误码） | `std::mutex` |
| `last_feedback_time_ns` / `feedback_received_once` | `std::atomic` |
| `Target_*` / `Kp_` / `Kd_` | 无（仅在主线程访问） |

---

## 安全机制

### 首帧同步

上电时目标位置默认为 0。若电机实际停在 3 rad 而 `kp=10`，
会输出 30 N·m（被限幅到 TMAX=10）→ 满力矩猛冲。

**处理：** 收到第一帧反馈时，将目标位置设为当前实际位置。

### 刚度恢复对齐

在速度/力矩模式下 `kp=0`，电机可能自由转动到很远的位置，而位置目标仍是旧值。
把 `kp` 改回非 0 的瞬间，位置误差可能达十几弧度，导致满力矩回冲。

**处理：** 检测到有效 `kp` 从 0 变为非 0 时，先把位置目标对齐当前位置。

### 反馈超时保护

超过 `timeout_ms`（默认 100 ms）无反馈时，切换为纯阻尼模式：

```
target_position = 当前位置
kp = 0
kd = 0.5
```

避免通信中断后电机保持错误的力矩输出。

### 力矩模式下的 kd 下限

`kp=0` 且 `kd=0` 时输出恒定力矩，空载情况下电机会持续加速（飞车）。

**处理：** 力矩模式下 `kd` 最小取 0.3。

---

## CAN FD 说明

本驱动使用 CAN FD 通信。相关的三个概念需要区分：

### FDF（帧格式位）

```
FDF = 0  → 经典 CAN 帧（≤ 8 字节）
FDF = 1  → CAN FD 帧（≤ 64 字节）
```

### BRS（速率切换位）

```
BRS = 0  → 整帧运行在仲裁段速率（格式是 FD，但没有提速）
BRS = 1  → 数据段切换到 canfd_baudrate，CRC 和 ACK 再切回仲裁段速率
```

**帧时序：**

```
┌──── 1 Mbps ────┐┌── 5 Mbps ──┐┌──── 1 Mbps ────┐
│ SOF 仲裁段 FDF  ││  8 字节数据 ││ CRC ACK EOF    │
│        BRS ESI  ││             ││                │
└─────────────────┘└─────────────┘└────────────────┘
                  ↑             ↑
            BRS=1 切换      切回仲裁段速率
```

### 通道级与帧级

SDK 中有两个独立的 `canfd` 标志：

| 层级 | 位置 | 作用 |
|---|---|---|
| 通道级 | `dmcan_channel_can_info_t.canfd` | 通道控制器是否工作在 FD 模式 |
| 帧级 | `dmcan_device_send_can(..., canfd, ...)` | 这一帧用 FD 还是经典格式 |

FD 模式控制器向下兼容经典帧，因此"通道开 FD 模式"不影响发送经典帧。

### 实测结果

| `canfd_channel` | `canfd_frame` | `canfd_brs` | 数据段 | 结果 |
|---|---|---|---|---|
| false | false | — | — | 通（经典 CAN） |
| true | false | — | — | 通（FD 通道 + 经典帧） |
| true | true | false | 1 Mbps | 通 |
| true | true | true | 5 Mbps | **通** |
| true | true | true | 2 Mbps | 不通 |

**前提：模块固件 ≥ v1.1.0.0。** 固件 v1.0.0.9 下 FD 帧无法收发。

---

## 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'dm4310_driver' not found` | 未 source 工作区 | `source install/setup.bash` |
| `ros2: 未找到命令` | 使用了 sudo | 不要用 sudo 运行 ROS |
| `没找到 LINKX4C` / `设备数 = 0` | 未插 / 被占用 / 权限 | 检查 `lsusb \| grep 34b7`，确认无其他进程占用 |
| `dmcan_device_open 失败`（设备数 = 1） | 设备被其他进程占用 | `pidof dm4310_node` 后 kill 残留进程 |
| `enable_channel 失败` | 模块进入异常状态 | **拔掉 USB 和 XT30 电源，等待 1 分钟**再上电 |
| `还没收到反馈` | 电机未上电 / CAN 未接 / ID 错误 | 检查 24V、CAN_H/L、`can_id`/`mst_id` |
| `数据传输错误`（DMTool） | CAN 总线无应答 | 检查 CAN 接线与电机供电 |
| 退出码 134 / `libusb_mutex_lock` 断言 | SDK 已知缺陷 | 忽略，不影响安全（失能帧已发出） |
| 电机不动也不硬 | 未使能 | 重启节点 |
| 电机猛冲 | kp 过大或目标阶跃过大 | 降低 `kp`，减小目标变化量 |
| 电机抖动 | kd 过小 | 增大 `kd` |

---

## 辅助工具

包内附带两个独立可执行程序（不依赖 ROS）：

```bash
# 验证 USB 与 SDK 能否找到设备并配置 4 个通道
./install/dm4310_driver/lib/dm4310_driver/sdk_probe

# 扫描 CAN_ID / MST_ID（未知 ID 时使用）
./install/dm4310_driver/lib/dm4310_driver/dm_scan
```

---

## 已知限制

| 项 | 说明 |
|---|---|
| 单电机 | 当前实现为单电机场景，多电机需扩展 `g_motor` 全局变量与回调分发 |
| 反馈格式 | 仅解析 MIT 反馈帧，不支持其他模式反馈 |
| 寄存器读写 | 未实现，需通过 DMTool 或单独工具 |
| SDK 退出崩溃 | `dmcan_context_destroy()` 在 `dmcan_device_close()` 后调用会触发 libusb 断言（SIGABRT）。当前实现跳过 destroy |
| 减速比 | 未在驱动内换算，`position`/`velocity` 为电机侧数值 |

---

## 安全提示

```
首次上电必须悬空固定电机
手始终放在 24V 电源开关旁
kp ≤ 50，kd ≤ 2
位置阶跃 ≤ 1.0 rad（安全阶跃 = TMAX / kp）
速度 ≤ 2.0 rad/s
力矩 ≤ 0.5 N·m
下电顺序：Ctrl+C 停节点 → 关 24V → 拔 USB
```

---

## 参考资料

- 达妙 DM-J4310 电机手册
- 达妙主协议文档 V1.4
- DM-LinkX-4C 模块使用说明书 V1.1
- ISO 11898-1:2015（CAN FD 标准）

---

## 许可

MIT
