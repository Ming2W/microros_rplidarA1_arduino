#ifndef CONFIG_H
#define CONFIG_H

#include <cstddef>
#include <cstdint>

// // ==================== 机器人运动学参数 ====================
// #define MOTOR_MAX_RPM 90            // 电机的最大转速（rpm）
// #define WHEEL_DIAMETER 0.08         // 机器人车轮直径（米）
// #define FR_WHEEL_DISTANCE 0.13      // 前后车轮之间的距离（米）
// #define LR_WHEEL_DISTANCE 0.195     // 左右车轮之间的距离（米）
// #define PWM_BITS 8                  // PWM 引脚分辨率

// // ==================== 控制参数 ====================
// constexpr size_t kWheelCount = 4;
// constexpr double kPulseDistanceMm = 0.16102564;   // 每个编码器脉冲对应的距离（mm）
// constexpr uint32_t kControlIntervalMs = 50;       // 控制周期（ms）
// constexpr uint32_t kTelemetryIntervalMs = 200;    // 遥测周期（ms）
// constexpr double kMaxTargetMmPerSec = 800.0;      // 最大目标速度（mm/s）
// constexpr double kWheelbaseFrontRear = 0.13;      // 前后轮中心间距（米）
// constexpr double kWheelbaseLeftRight = 0.195;     // 左右轮中心间距（米）
// constexpr double kEncoderPulsesPerRev = 1560.0;   // 编码器每转脉冲数

// // ==================== PID 默认参数 ====================
// constexpr double kDefaultKp = 0.15;
// constexpr double kDefaultKi = 0.5;
// constexpr double kDefaultKd = 0.003;

// // ==================== 轮速一致性反馈 ====================
// constexpr double kDefaultKc = 0.008;              // 一致性系数：让四轮趋向同步
// constexpr bool kEnableConsistency = true;       // 启用轮速一致性反馈

// // ==================== 前馈参数 ====================
// constexpr double kFeedforwardGain = 0.0;          // 前馈增益：暂时关闭
// constexpr bool kEnableFeedforward = false;        // 关闭前馈控制

// // ==================== 机体速度斜坡参数（公共斜坡） ====================
// constexpr double kBodyAccelXY = 0.3;              // 线速度斜坡 (m/s²)
// constexpr double kBodyAccelW = 1.0;               // 角速度斜坡 (rad/s²)
// constexpr bool kEnableBodyRamp = true;            // 启用机体速度斜坡

// // ==================== PWM 配置 ====================
// constexpr int kPwmFreq = 20000;
// constexpr int kPwmResolution = 8;

// // ==================== I2C 配置 ====================
// constexpr uint32_t kI2cFreq = 100000;

// ==================== micro-ROS 配置 ====================
#define WIFI_SSID "YOUR_COMPUTER"
#define WIFI_PASSWORD "********"
#define AGENT_IP "192.168.***.***"
#define AGENT_PORT 8888

// ==================== 激光雷达配置 ====================
#define LIDAR_BAUD_RATE 115200           // RPLIDAR A1 波特率
#define LIDAR_RANGE_MIN 0.15f            // 最小测量距离 (m)
#define LIDAR_RANGE_MAX 12.0f            // 最大测量距离 (m)
#define LIDAR_FRAME_ID "laser_frame"     // TF 坐标系名称
#define LIDAR_UPDATE_INTERVAL_MS 20      // 数据更新间隔 (ms)
#define LIDAR_PUBLISH_INTERVAL_MS 100    // 发布间隔 (ms)，约10Hz

#endif // CONFIG_H
