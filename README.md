# CJ02-IMU

**迷你六轴 IMU 传感器 · ESKF 姿态解算 · 800 Hz 输出**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Dashboard](https://img.shields.io/badge/Dashboard-GitHub%20Pages-blue)](https://creavisiontech.github.io/CJ02-IMU/)
[![Protocol](https://img.shields.io/badge/Protocol-v1.0-green)](docs/PROTOCOL.md)

---

## 简介

CJ02-IMU 是一款高性能迷你六轴惯性测量单元（IMU），内置 ESKF（误差状态卡尔曼滤波器）实时姿态解算，以 **800 Hz** 频率通过 UART 输出原始传感器数据和融合姿态角。

### 核心特性

| 特性 | 规格 |
|------|------|
| 姿态输出频率 | 800 Hz |
| 内部采样频率 | 1600 Hz |
| 加速度计量程 | ±2 g |
| 陀螺仪量程 | ±2000 °/s |
| 静止漂移（roll/pitch） | < 0.05°/min |
| 静止漂移（yaw） | < 0.1°/min |
| 外部触发同步精度 | 0.625 ms |
| 通信接口 | UART 460800 bps 8N1 |
| 供电 | 3.3V |

### 在线工具

- **[Web Dashboard](https://creavisiontech.github.io/CJ02-IMU/)** — 浏览器直接连接串口，实时查看 3D 姿态、原始数据、曲线图，可在线修改传感器配置参数

---

## 快速上手

### 方式一：Python（最简单）

```bash
# 安装依赖
pip install pyserial

# 运行示例（Linux）
python sdk/python/example.py /dev/ttyUSB0

# 运行示例（Windows）
python sdk/python/example.py COM11
```

输出：
```
Connected to CJ02-IMU on /dev/ttyUSB0 @ 460800 baud
R=  12.34°  P=  -5.67°  Y=  89.12°  mode=RUN     zaru=True  static=True  [798 Hz]
```

### 方式二：C++

```bash
cd sdk/cpp
mkdir build && cd build
cmake .. && make

# 运行
./cj02_example /dev/ttyUSB0     # Linux
./cj02_example.exe COM11        # Windows
```

### 方式三：Web Dashboard（零安装）

1. 用 Chrome 或 Edge 打开 [在线 Dashboard](https://creavisiontech.github.io/CJ02-IMU/)
2. 点击「连接串口」选择对应 COM 口
3. 即可看到实时 3D 姿态、原始数据、曲线图

### 方式四：ROS1

```bash
# 复制到 catkin 工作空间
cp -r ros/cj02_imu_node ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make

# 运行
source devel/setup.bash
rosrun cj02_imu_node cj02_imu_node _port:=/dev/ttyUSB0

# 查看话题
rostopic hz /imu/data        # 应显示 ~800 Hz
rostopic echo /imu/attitude  # 欧拉角
```

### 方式五：ROS2

```bash
# 复制到 colcon 工作空间
cp -r ros2/cj02_imu ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select cj02_imu

# 运行
source install/setup.bash
ros2 run cj02_imu cj02_imu_node --ros-args -p port:=/dev/ttyUSB0

# 查看话题
ros2 topic hz /imu/data
ros2 topic echo /imu/attitude
```

---

## 项目结构

```
CJ02-IMU/
├── README.md                 # 本文件
├── LICENSE                   # MIT 许可证
├── docs/
│   └── PROTOCOL.md           # 通信协议详细文档
├── sdk/
│   ├── cpp/
│   │   ├── include/
│   │   │   └── cj02_imu.h    # C++ SDK（header-only）
│   │   ├── src/
│   │   │   └── example.cpp   # C++ 示例程序
│   │   └── CMakeLists.txt
│   └── python/
│       ├── cj02_imu.py       # Python SDK
│       ├── example.py        # Python 示例
│       └── requirements.txt
├── ros/
│   └── cj02_imu_node/        # ROS1 驱动节点
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/cj02_imu_node.cpp
├── ros2/
│   └── cj02_imu/             # ROS2 驱动节点
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/cj02_imu_node.cpp
├── dashboard/
│   └── index.html            # Web Dashboard（GitHub Pages 部署）
└── .github/
    └── workflows/
        └── pages.yml         # GitHub Pages 自动部署
```

---

## 数据输出

CJ02-IMU 以 800 Hz 输出两种数据帧（详见 [通信协议](docs/PROTOCOL.md)）：

| 数据 | 帧头 | 内容 | 频率 |
|------|------|------|------|
| 原始数据 | `0xAA` | 加速度 (mg) + 陀螺仪 (°/s) | 800 Hz |
| 姿态角 | `0xAB` | roll/pitch/yaw (°) + 模式 + 标志 | 800 Hz |
| 同步事件 | `0xAD` | 外部触发时间戳 + 当时姿态 | 异步 |
| 配置 | `0xAC` | 参数读写命令/回复 | 按需 |

**Python 快速读取：**

```python
from cj02_imu import CJ02IMU

imu = CJ02IMU()
imu.on_attitude = lambda f: print(f"roll={f.roll:.2f}°")
imu.open("/dev/ttyUSB0")
imu.run()
```

**C++ 快速读取：**

```cpp
#include "cj02_imu.h"
cj02::CJ02IMU imu;
imu.onAttitude([](const cj02::AttitudeFrame& f) {
    printf("roll=%.2f\n", f.roll);
});
imu.open("/dev/ttyUSB0");
imu.run();
```

---

## ROS 话题

ROS1 和 ROS2 节点发布相同的话题：

| 话题 | 消息类型 | 内容 | 频率 |
|------|---------|------|------|
| `/imu/data_raw` | `sensor_msgs/Imu` | 原始加速度 + 角速度 | 800 Hz |
| `/imu/data` | `sensor_msgs/Imu` | 含姿态四元数 | 800 Hz |
| `/imu/attitude` | `geometry_msgs/Vector3Stamped` | 欧拉角 (roll/pitch/yaw, °) | 800 Hz |

---

## 外部触发同步

CJ02-IMU 支持外部传感器触发同步功能：

1. 将外部传感器（如相机）的触发信号连接到传感器的 SYNC 输入引脚
2. 每次触发脉冲到来时，传感器会插入一个 `0xAD` 同步帧到数据流
3. 同步帧包含触发时刻的内部采样计数器（1600 Hz 分辨率）和当时的姿态角
4. 上位机可通过 `trigger_seq` 精确对齐外部事件与 IMU 数据（精度 0.625 ms）

**Python 监听同步事件：**

```python
imu.on_sync = lambda f: print(f"外部触发! seq={f.trigger_seq} R={f.roll:.1f}°")
```

---

## 配置参数

通过串口命令（`0xAC` 帧）可在线修改传感器参数并持久化到内部 Flash（断电不丢失）。支持的参数包括：

- ESKF 滤波器参数（陀螺噪声、零偏漂移、ZARU 方差等 17 项）
- 静止检测阈值
- 触发输出分频和占空比

详见 [通信协议 §5 配置参数](docs/PROTOCOL.md#5-配置参数80-字节负载)。

---

## 文档

- [通信协议](docs/PROTOCOL.md) — 帧格式、校验算法、配置命令、参数表

---

## 许可证

[MIT License](LICENSE)

---

## 购买与技术支持

- 官方网站：[Creavision Tech](https://creavision.tech)
- 技术支持：support@creavision.tech
