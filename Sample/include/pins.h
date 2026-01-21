#ifndef PINS_H
#define PINS_H

#include "config.h"

// ==================== 编码器引脚 ====================
constexpr int ENC1_A = 4;
constexpr int ENC1_B = 5;
constexpr int ENC2_A = 6;
constexpr int ENC2_B = 7;
constexpr int ENC3_A = 41;
constexpr int ENC3_B = 40;
constexpr int ENC4_A = 39;
constexpr int ENC4_B = 38;

// ==================== I2C 引脚 ====================
constexpr int I2C_SDA = 15;
constexpr int I2C_SCL = 16;

// ==================== PWM 引脚 ====================
extern const int pwmPins[kWheelCount];
extern const int pwmChannels[kWheelCount];

// ==================== 激光雷达引脚 ====================
// 注意：这里是 ESP32 的引脚功能定义
// 如果雷达TX连到GPIO17，则GPIO17是ESP32的RX
// 如果雷达RX连到GPIO18，则GPIO18是ESP32的TX
constexpr int LIDAR_TX_PIN = 18;         // ESP32 TX -> 雷达 RX (GPIO18)
constexpr int LIDAR_RX_PIN = 17;         // ESP32 RX <- 雷达 TX (GPIO17)
constexpr int LIDAR_MOTOR_PWM_PIN = 9;   // 雷达电机 PWM 控制

#endif // PINS_H
