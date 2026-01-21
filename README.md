# RPLIDAR A1 micro-ROS Driver for ESP32S3

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## English

A lightweight RPLIDAR A1 driver implementation for ESP32S3 using PlatformIO Arduino framework with micro-ROS support. This project enables publishing LaserScan messages to ROS2 via micro-ROS.

### ✨ Features

- 🚀 **Lightweight**: Pure C/C++ implementation without SLAMTEC SDK dependency
- 📡 **micro-ROS Integration**: Publishes `sensor_msgs/LaserScan` to `/scan` topic
- ⚡ **Real-time Processing**: FreeRTOS tasks for non-blocking data handling
- 🔧 **Configurable**: Easy pin and parameter configuration
- 📊 **Debug Support**: Built-in serial debug output

### 📋 Hardware Requirements

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3 (recommended: ESP32-S3-DevKitC-1) |
| Lidar | SLAMTEC RPLIDAR A1 (115200 baud) |
| Power | 5V for Lidar, 3.3V for ESP32 |

### 🔌 Wiring Diagram

```
ESP32S3              RPLIDAR A1
─────────            ──────────
GPIO17 (RX) ◄─────── TX (Green)
GPIO18 (TX) ───────► RX (White)  
GPIO9  (PWM) ──────► MOTOCTL (Blue)
GND ────────────────  GND (Black)
5V ─────────────────  VCC (Red)
```

> **Note**: The TX/RX naming refers to ESP32's perspective. Connect Lidar's TX to ESP32's RX pin.

### 📦 Dependencies

Add to `platformio.ini`:

```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_microros_transport = wifi
lib_deps = 
    https://gitee.com/magicedge/micro_ros_platformio.git
```

### 🚀 Quick Start

#### 1. Clone the Repository

```bash
git clone https://github.com/Ming2W/microros_rplidarA1_arduino.git
cd microros_rplidarA1_arduino
```

#### 2. Configure WiFi and Agent

Edit `include/config.h`:

```c
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
#define AGENT_IP "192.168.1.100"  // micro-ROS agent IP
#define AGENT_PORT 8888
```

#### 3. Configure Pins (if needed)

Edit `include/pins.h`:

```c
constexpr int LIDAR_TX_PIN = 18;         // ESP32 TX -> Lidar RX
constexpr int LIDAR_RX_PIN = 17;         // ESP32 RX <- Lidar TX
constexpr int LIDAR_MOTOR_PWM_PIN = 9;   // Motor PWM control
```

#### 4. Build and Upload

After adding these codes to your environment, compile and upload them
```bash
pio run --target upload
```

#### 5. Start micro-ROS Agent

On your ROS2 machine:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### 6. Verify Data

```bash
# List topics
ros2 topic list

# View scan data
ros2 topic echo /scan

# Visualize in RViz2
rviz2
# Add LaserScan display, set topic to /scan, fixed frame to laser_frame
```

### 📁 Project Structure

```
microros_rplidar/
├── include/
│   ├── config.h              # Configuration parameters
│   └── pins.h                # Hardware pin definitions
├── src/
│   ├── main.cpp              # Main program
│   ├── rplidar_a1.h          # Driver header (protocol & API)
│   ├── rplidar_a1.cpp        # Driver implementation
│   ├── lidar_publisher.h     # ROS publisher header
│   └── lidar_publisher.cpp   # ROS publisher implementation
└── platformio.ini            # PlatformIO configuration
```

### ⚙️ Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LIDAR_BAUD_RATE` | 115200 | Serial baud rate |
| `LIDAR_RANGE_MIN` | 0.15m | Minimum valid range |
| `LIDAR_RANGE_MAX` | 12.0m | Maximum valid range |
| `LIDAR_FRAME_ID` | "laser_frame" | TF frame ID |
| `LIDAR_PUBLISH_INTERVAL_MS` | 100 | Publish interval (~10Hz) |

### 📊 Data Flow

```
RPLIDAR A1 ──UART──► ESP32 Serial2 ──► rplidar_task (parse)
                                              │
                                              ▼
                                    tempPoints[] buffer
                                              │
                                        startBit=1?
                                              │
                                              ▼
                                    processCompleteScan()
                                              │
                                              ▼
                                    completedScan[360]
                                              │
                    ┌─────────────────────────┴─────────────────────────┐
                    ▼                                                   ▼
          lidar_update_task                                    publishLidar()
          (convert to ROS msg)                                 (publish /scan)
                    │                                                   │
                    └───────────────────────────────────────────────────┘
                                              │
                                              ▼
                                    micro-ROS Agent (UDP)
                                              │
                                              ▼
                                    ROS2 /scan Topic
```

### 🔬 Protocol Details

#### Command Format
```
[0xA5] [CMD]
Example: 0xA5 0x20 = Start Scan
```

#### Response Header (7 bytes)
```
[0xA5] [0x5A] [size:4] [type:1]
Scan response: A5 5A 05 00 00 40 81
```

#### Measurement Node (5 bytes)
```c
struct {
    uint8_t  sync_quality;      // bit0:sync, bit1:~sync, bit2-7:quality
    uint16_t angle_q6_checkbit; // bit0:check(=1), bit1-15:angle*64
    uint16_t distance_q2;       // distance * 4 (unit: 0.25mm)
}
```

#### Data Conversion
```c
angle    = (angle_q6_checkbit >> 1) / 64.0f;  // degrees
distance = distance_q2 / 4.0f;                 // mm
quality  = sync_quality >> 2;                  // 0-63
startBit = sync_quality & 0x01;                // new scan flag
```

### 📈 Performance

| Metric | Value |
|--------|-------|
| Scan Frequency | ~5 Hz |
| Points per Scan | ~400 |
| Valid Points | ~200+ |
| Publish Rate | 10 Hz |
| Angular Resolution | 1° |

### 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| No data received | Check TX/RX wiring (swap if needed) |
| Motor not spinning | Verify PWM pin and 5V power |
| micro-ROS connection failed | Check WiFi and Agent IP/port |
| Stack overflow crash | Increase task stack size (8192+) |
| Few valid points | Check for obstacles or sensor cleaning |

### 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### 🙏 Acknowledgments

- [SLAMTEC](https://www.slamtec.com/) for RPLIDAR protocol documentation
- [micro-ROS](https://micro.ros.org/) for embedded ROS2 support
- [PlatformIO](https://platformio.org/) for build system

---

<a name="中文"></a>
## 中文

基于 ESP32S3 PlatformIO Arduino 框架实现的轻量级 RPLIDAR A1 驱动，支持 micro-ROS，可将 LaserScan 消息发布到 ROS2。

### ✨ 特性

- 🚀 **轻量级**: 纯 C/C++ 实现，无需 SLAMTEC SDK
- 📡 **micro-ROS 集成**: 发布 `sensor_msgs/LaserScan` 到 `/scan` 话题
- ⚡ **实时处理**: FreeRTOS 任务实现非阻塞数据处理
- 🔧 **可配置**: 简单的引脚和参数配置
- 📊 **调试支持**: 内置串口调试输出

### 📋 硬件需求

| 组件 | 规格 |
|------|------|
| MCU | ESP32-S3 (推荐: ESP32-S3-DevKitC-1) |
| 雷达 | SLAMTEC RPLIDAR A1 (115200 波特率) |
| 电源 | 雷达需要 5V，ESP32 需要 3.3V |

### 🔌 接线图

```
ESP32S3              RPLIDAR A1
─────────            ──────────
GPIO17 (RX) ◄─────── TX (绿线)
GPIO18 (TX) ───────► RX (白线)  
GPIO9  (PWM) ──────► MOTOCTL (蓝线)
GND ────────────────  GND (黑线)
5V ─────────────────  VCC (红线)
```

> **注意**: TX/RX 命名是从 ESP32 角度来看的。将雷达的 TX 连接到 ESP32 的 RX 引脚。

### 📦 依赖

在 `platformio.ini` 中添加:

```ini
[env:esp32s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_microros_transport = wifi
lib_deps = 
    https://gitee.com/magicedge/micro_ros_platformio.git
```

### 🚀 快速开始

#### 1. 克隆仓库

```bash
git clone https://github.com/Ming2W/microros_rplidarA1_arduino.git
cd microros_rplidarA1_arduino
```

#### 2. 配置 WiFi 和 Agent

编辑 `include/config.h`:

```c
#define WIFI_SSID "你的WiFi名称"
#define WIFI_PASSWORD "你的WiFi密码"
#define AGENT_IP "192.168.1.100"  // micro-ROS agent IP
#define AGENT_PORT 8888
```

#### 3. 配置引脚（如需修改）

编辑 `include/pins.h`:

```c
constexpr int LIDAR_TX_PIN = 18;         // ESP32 TX -> 雷达 RX
constexpr int LIDAR_RX_PIN = 17;         // ESP32 RX <- 雷达 TX
constexpr int LIDAR_MOTOR_PWM_PIN = 9;   // 电机 PWM 控制
```

#### 4. 编译和上传

将这些代码加入到你的环境中后，编译和上传
```bash
pio run --target upload
```

#### 5. 启动 micro-ROS Agent

在 ROS2 电脑上:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### 6. 验证数据

```bash
# 查看话题
ros2 topic list

# 查看扫描数据
ros2 topic echo /scan

# 在 RViz2 中可视化
rviz2
# 添加 LaserScan 显示，设置话题为 /scan，固定坐标系为 laser_frame
```

### 📁 项目结构

```
microros_rplidar/
├── include/
│   ├── config.h              # 配置参数
│   └── pins.h                # 硬件引脚定义
├── src/
│   ├── main.cpp              # 主程序
│   ├── rplidar_a1.h          # 驱动头文件（协议和 API）
│   ├── rplidar_a1.cpp        # 驱动实现
│   ├── lidar_publisher.h     # ROS 发布器头文件
│   └── lidar_publisher.cpp   # ROS 发布器实现
└── platformio.ini            # PlatformIO 配置
```

### ⚙️ 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LIDAR_BAUD_RATE` | 115200 | 串口波特率 |
| `LIDAR_RANGE_MIN` | 0.15m | 最小有效距离 |
| `LIDAR_RANGE_MAX` | 12.0m | 最大有效距离 |
| `LIDAR_FRAME_ID` | "laser_frame" | TF 坐标系名称 |
| `LIDAR_PUBLISH_INTERVAL_MS` | 100 | 发布间隔（约 10Hz）|

### 📊 数据流程

```
RPLIDAR A1 ──UART──► ESP32 Serial2 ──► rplidar_task (解析)
                                              │
                                              ▼
                                    tempPoints[] 缓冲区
                                              │
                                        startBit=1?
                                        (新一圈标志)
                                              │
                                              ▼
                                    processCompleteScan()
                                              │
                                              ▼
                                    completedScan[360]
                                              │
                    ┌─────────────────────────┴─────────────────────────┐
                    ▼                                                   ▼
          lidar_update_task                                    publishLidar()
          (转换为 ROS 消息)                                    (发布 /scan)
                    │                                                   │
                    └───────────────────────────────────────────────────┘
                                              │
                                              ▼
                                    micro-ROS Agent (UDP)
                                              │
                                              ▼
                                    ROS2 /scan 话题
```

### 📈 性能指标

| 指标 | 数值 |
|------|------|
| 扫描频率 | ~5 Hz |
| 每圈点数 | ~400 |
| 有效点数 | ~200+ |
| 发布频率 | 10 Hz |
| 角度分辨率 | 1° |

### 🐛 故障排除

| 问题 | 解决方案 |
|------|----------|
| 没有接收到数据 | 检查 TX/RX 接线（尝试交换）|
| 电机不转 | 检查 PWM 引脚和 5V 供电 |
| micro-ROS 连接失败 | 检查 WiFi 和 Agent IP/端口 |
| 栈溢出崩溃 | 增加任务栈大小（8192+）|
| 有效点数过少 | 检查障碍物或清洁传感器 |

### 📄 许可证

本项目使用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

### 🙏 致谢

- [SLAMTEC](https://www.slamtec.com/) 提供 RPLIDAR 协议文档
- [micro-ROS](https://micro.ros.org/) 提供嵌入式 ROS2 支持
- [PlatformIO](https://platformio.org/) 提供构建系统

---

## 📬 Contact

If you have any questions or suggestions, please open an issue or submit a PR.

**Happy Mapping! 🗺️**
