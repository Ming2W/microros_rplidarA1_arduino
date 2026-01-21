/**
 * @file rplidar_a1.h
 * @brief SLAMTEC RPLIDAR A1 驱动头文件
 * 
 * 基于 SLAMTEC RPLIDAR 协议实现，用于 ESP32S3 Arduino 平台
 * 参考 MS200 驱动实现方式
 */

#ifndef RPLIDAR_A1_H
#define RPLIDAR_A1_H

#include <Arduino.h>

// ==================== 协议常量定义 ====================
// 命令同步字节
#define RPLIDAR_CMD_SYNC_BYTE           0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD     0x80

// 响应同步字节
#define RPLIDAR_ANS_SYNC_BYTE1          0xA5
#define RPLIDAR_ANS_SYNC_BYTE2          0x5A

// 命令定义
#define RPLIDAR_CMD_STOP                0x25
#define RPLIDAR_CMD_RESET               0x40
#define RPLIDAR_CMD_SCAN                0x20
#define RPLIDAR_CMD_FORCE_SCAN          0x21
#define RPLIDAR_CMD_GET_DEVICE_INFO     0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH   0x52

// 响应类型
#define RPLIDAR_ANS_TYPE_DEVINFO        0x04
#define RPLIDAR_ANS_TYPE_DEVHEALTH      0x06
#define RPLIDAR_ANS_TYPE_MEASUREMENT    0x81

// 测量数据标志
#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1 << 0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1 << 0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

// 健康状态
#define RPLIDAR_STATUS_OK               0x0
#define RPLIDAR_STATUS_WARNING          0x1
#define RPLIDAR_STATUS_ERROR            0x2

// 雷达参数
#define RPLIDAR_POINT_MAX               360     // 最大点数（一圈360度）
#define RPLIDAR_RX_BUF_SIZE             256     // 接收缓冲区大小

// ==================== 数据结构定义 ====================

// 响应头结构
typedef struct __attribute__((packed)) {
    uint8_t syncByte1;      // 必须是 0xA5
    uint8_t syncByte2;      // 必须是 0x5A
    uint32_t size_q30_subtype;  // size:30, subType:2
    uint8_t type;
} rplidar_ans_header_t;

// 设备信息响应
typedef struct __attribute__((packed)) {
    uint8_t model;
    uint16_t firmware_version;
    uint8_t hardware_version;
    uint8_t serialnum[16];
} rplidar_response_device_info_t;

// 设备健康状态响应
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint16_t error_code;
} rplidar_response_device_health_t;

// 单点测量数据（协议原始格式）
typedef struct __attribute__((packed)) {
    uint8_t sync_quality;       // syncbit:1, syncbit_inverse:1, quality:6
    uint16_t angle_q6_checkbit; // check_bit:1, angle_q6:15
    uint16_t distance_q2;       // 距离（单位：0.25mm）
} rplidar_response_measurement_node_t;

// 单点数据（解析后）
typedef struct {
    float angle;        // 角度（度）
    float distance;     // 距离（mm）
    uint8_t quality;    // 信号质量
    bool startBit;      // 是否是新一圈的起点
} rplidar_point_t;

// 一圈完整扫描数据
typedef struct {
    rplidar_point_t points[RPLIDAR_POINT_MAX];
    uint16_t count;         // 实际点数
    bool valid;             // 数据是否有效
    uint32_t timestamp;     // 时间戳
} rplidar_scan_data_t;

// 驱动状态
typedef enum {
    RPLIDAR_STATE_IDLE = 0,
    RPLIDAR_STATE_SCANNING,
    RPLIDAR_STATE_PROCESSING,
    RPLIDAR_STATE_ERROR
} rplidar_state_t;

// ==================== 函数声明 ====================

/**
 * @brief 初始化 RPLIDAR A1 驱动
 * @param serial 用于通信的 HardwareSerial 指针
 * @param rxPin 串口 RX 引脚
 * @param txPin 串口 TX 引脚
 * @param motorPwmPin 电机 PWM 控制引脚
 * @return true 初始化成功
 */
bool rplidar_init(HardwareSerial *serial, int rxPin, int txPin, int motorPwmPin);

/**
 * @brief 启动雷达电机
 */
void rplidar_start_motor();

/**
 * @brief 停止雷达电机
 */
void rplidar_stop_motor();

/**
 * @brief 设置电机 PWM 占空比
 * @param duty 占空比 0-255
 */
void rplidar_set_motor_pwm(uint8_t duty);

/**
 * @brief 发送停止扫描命令
 */
void rplidar_stop_scan();

/**
 * @brief 发送开始扫描命令
 * @return true 命令发送成功
 */
bool rplidar_start_scan();

/**
 * @brief 发送复位命令
 */
void rplidar_reset();

/**
 * @brief 获取设备信息
 * @param info 设备信息输出
 * @return true 获取成功
 */
bool rplidar_get_device_info(rplidar_response_device_info_t *info);

/**
 * @brief 获取设备健康状态
 * @param health 健康状态输出
 * @return true 获取成功
 */
bool rplidar_get_health(rplidar_response_device_health_t *health);

/**
 * @brief 处理串口接收数据（在循环中调用）
 * 内部状态机解析协议数据
 */
void rplidar_process_data();

/**
 * @brief 检查是否有新的完整一圈数据
 * @return true 有新数据
 */
bool rplidar_has_new_scan();

/**
 * @brief 清除新数据标志
 */
void rplidar_clear_new_scan_flag();

/**
 * @brief 获取扫描数据
 * @param data 扫描数据输出
 */
void rplidar_get_scan_data(rplidar_scan_data_t *data);

/**
 * @brief 获取指定角度的距离
 * @param angle 角度 (0-359)
 * @return 距离（mm），0 表示无效
 */
uint16_t rplidar_get_distance(uint16_t angle);

/**
 * @brief 获取指定角度的信号质量
 * @param angle 角度 (0-359)
 * @return 信号质量 (0-63)
 */
uint8_t rplidar_get_quality(uint16_t angle);

/**
 * @brief 获取当前驱动状态
 * @return 驱动状态
 */
rplidar_state_t rplidar_get_state();

/**
 * @brief 雷达数据处理任务（FreeRTOS 任务函数）
 * @param arg 任务参数
 */
void rplidar_task(void *arg);

/**
 * @brief 创建雷达处理任务
 * @param stackSize 任务栈大小
 * @param priority 任务优先级
 * @param coreId 绑定的 CPU 核心 (-1 表示不绑定)
 * @return true 创建成功
 */
bool rplidar_create_task(uint32_t stackSize = 8192, uint8_t priority = 10, int coreId = 1);

#endif // RPLIDAR_A1_H
