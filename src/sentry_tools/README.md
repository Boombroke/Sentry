# sentry_tools — 哨兵机器人调试工具集

## 快速开始

```bash
# 工具箱（串口 Mock + 地图拾取 + 连通性检测），无需 ROS
python3 src/sentry_tools/sentry_toolbox.py

# 串口数据可视化（需 ROS 环境）
source install/setup.bash
python3 src/sentry_tools/serial_visualizer.py
```

依赖：`pip install pyserial`（PyQt5/matplotlib/numpy/pyyaml 随 ROS2 Humble 自带）

---

## 工具箱 (sentry_toolbox.py)

三个标签页，暗色主题，无需 ROS 即可运行。

### 标签页 1：串口 Mock

模拟电控向上位机发送协议包，用于脱离硬件调试。

**操作步骤：**

1. 顶栏选择串口和波特率，点击 Connect
2. 在 IMU/Status/HP 子页签调整字段值
3. 勾选 Enable 并设置发送周期
4. 底部「接收显示」实时显示 ROS 回传的速度指令

**发送的包：**

| 标签页 | 帧头 | 默认周期 | 可调字段 |
|---|---|---|---|
| IMU | 0xA1 | 20ms | pitch, yaw（滑块 ±3.14） |
| Status | 0xA2 | 100ms | 比赛阶段(下拉)、剩余时间、血量、弹量、红蓝队(单选)、RFID(勾选) |
| HP | 0xA3 | 500ms | 7 个己方血量（1/2/3/4/7号 + 前哨 + 基地） |

**控件是动态生成的**——修改 `protocol.yaml` 并重新生成后，新字段会自动出现。

**虚拟串口联调：**

```bash
# 终端 1：创建虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 输出 /dev/pts/3 和 /dev/pts/4

# 终端 2：工具箱连接一端
python3 src/sentry_tools/sentry_toolbox.py
# 选择 /dev/pts/3，Connect

# 终端 3：被测程序连接另一端
ros2 launch serial_driver serial_driver.launch.py device_name:=/dev/pts/4
```

### 标签页 2：地图坐标拾取

加载地图文件，鼠标点击拾取导航目标点坐标。

**操作步骤：**

1. 点击「选择地图」，选择 `.yaml` 地图文件（如 `sentry_nav_bringup/map/rmul_2026.yaml`）
2. 地图渲染后，左键点击标记坐标点（绿色 X）
3. 右侧列表显示所有已拾取坐标
4. 选中列表项，点击「复制选中」复制到剪贴板
5. 「清除标记」清空地图标记 + 列表

拾取的坐标可直接填入行为树的目标位置配置。

### 标签页 3：连通性检测

一键检测系统各链路是否正常。

**运行模式：**

| 模式 | 条件 | 检测项 |
|---|---|---|
| 基础模式 | 默认 | 串口设备 |
| ROS 模式 | 已 source ROS 环境 | 串口 + 节点 + Topic + TF |

**操作步骤：**

1. 点击「立即刷新」执行一次检测
2. 勾选「自动刷新」开启定时检测（默认 5 秒）
3. 勾选「详细频率检测」获取 Topic 实际频率（较慢）

**检测项：**

| 分组 | 检测内容 | 状态 |
|---|---|---|
| 串口设备 | `/dev/ttyACM*`, `/dev/ttyUSB*` | ✅ 存在 / ❌ 无设备 |
| 关键节点 | serial_driver, controller_server, planner_server, bt_navigator, behavior_server, loam_interface 等 | ✅ 运行中 / ❌ 未检测到 |
| Topic 状态 | gimbal_joint_state, cmd_vel, odometry, obstacle_scan, terrain_map, referee/* | ✅ 有发布者 / ⚠️ 频率低 / ❌ 无发布者 |
| TF 链路 | map→odom, odom→chassis, chassis→gimbal_yaw | ✅ 正常 / ❌ 断开 |

**底部汇总**：`总览: 12/15 正常  ⚠️ 1 警告  ❌ 2 异常`

**一键修复：**

检测失败时，每个异常项旁边会显示：
- 灰色提示文字（问题原因 + 修复建议）
- 蓝色「修复」按钮（点击后在终端执行修复脚本）

顶部「一键修复」按钮执行全量环境配置（`setup_env.sh`）。

**修复脚本对照：**

| 问题 | 修复脚本 | 作用 |
|---|---|---|
| ROS2 未安装 | `bash src/scripts/fix_ros_env.sh` | 安装 ROS2 Humble |
| Nav2 节点缺失 | `bash src/scripts/fix_nav2_deps.sh` | 安装 Nav2 + 导航依赖 |
| 串口无权限 | `bash src/scripts/fix_serial_permission.sh` | 加入 dialout 组 + 设置设备权限 |
| 编译问题 | `bash src/scripts/fix_build.sh` | 清理缓存 + rosdep + 全量重编 |
| 以上全部 | `bash src/scripts/setup_env.sh` | 一键全量环境配置 |

**典型排查流程：**

| 现象 | 看哪里 | 修复 |
|---|---|---|
| 机器人不动 | `/cmd_vel` topic + controller_server | 启动导航 launch |
| 导航无路径 | planner_server + obstacle_scan + terrain_map | `fix_nav2_deps.sh` + 启动 launch |
| 定位漂移 | `odom→chassis` TF + odometry topic | 检查 point_lio 节点 |
| 裁判系统异常 | 串口设备 + `referee/*` topic | `fix_serial_permission.sh` |
| 行为树不执行 | sentry_behavior_server + referee topic | `fix_build.sh` |
| 所有都红 | 全部 | `setup_env.sh`（一键修复按钮）|

---

## 串口数据可视化 (serial_visualizer.py)

订阅 ROS topic 实时显示串口链路数据。暗色主题，类似轻量版 Foxglove。

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/sentry_tools/serial_visualizer.py
```

**需要先启动导航栈或 serial_driver 节点。**

| 区域 | 内容 |
|---|---|
| 左上 | 云台 pitch/yaw 滚动曲线（10s 窗口） |
| 左下 | 导航速度 vx/vy/vw 滚动曲线 |
| 右上 | 比赛阶段 + 倒计时进度条 |
| 右中 | 自身 HP 进度条（绿/黄/红）+ 弹量大字 |
| 右下 | 全队 7 条 HP 柱状条 |
| 底部 | 各 topic 收包状态 ✓/✗ + UI 刷新率 |

**完整联调 4 终端：**

```bash
# A: 虚拟串口
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# B: ROS 串口驱动
ros2 launch serial_driver serial_driver.launch.py device_name:=/dev/pts/4

# C: 串口 Mock（工具箱连另一端）
python3 src/sentry_tools/sentry_toolbox.py

# D: 数据可视化
python3 src/sentry_tools/serial_visualizer.py
```

---

## 协议扩展

串口协议定义在 `src/serial/serial_driver/protocol/protocol.yaml`。

**修改协议的流程：**

```bash
# 1. 编辑唯一真相源
vim src/serial/serial_driver/protocol/protocol.yaml

# 2. 生成代码
cd src/serial/serial_driver/protocol && python3 generate.py

# 3. 部署到各位置
cp generated/packet.hpp ../include/rm_serial_driver/
cp generated/navigation_auto.h ../example/
cp generated/protocol.py ../../../sentry_tools/

# 4. 编译 ROS 端
cd /path/to/workspace && colcon build --packages-select rm_serial_driver
```

工具箱 GUI 控件自动跟随 protocol.py 变化，无需改代码。

---

## 文件结构

```text
src/sentry_tools/
├── sentry_toolbox.py          # 工具箱（串口 Mock + 地图拾取 + 连通性检测）
├── serial_visualizer.py       # 串口数据实时可视化（需 ROS）
├── protocol.py                # 协议定义（由 generate.py 生成，勿手动编辑）
└── README.md

src/serial/serial_driver/protocol/
├── protocol.yaml              # 协议唯一真相源
├── generate.py                # 代码生成器
├── templates/                 # Jinja2 模板
│   ├── packet.hpp.j2
│   ├── navigation_auto.h.j2
│   └── protocol_py.j2
└── generated/                 # 生成产物
```
