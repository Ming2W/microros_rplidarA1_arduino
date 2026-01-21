/**
 * @file lidar_publisher.cpp
 * @brief 激光雷达 micro-ROS LaserScan 发布器实现
 */

#include <Arduino.h>
#include <math.h>
#include <cstring>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>
#include "lidar_publisher.h"
#include "rplidar_a1.h"
#include "config.h"
#include "pins.h"

// ==================== 激光雷达发布器 ====================
rcl_publisher_t lidarPublisher;
sensor_msgs__msg__LaserScan lidarMsg;
bool lidarPublisherReady = false;

// 任务句柄
static TaskHandle_t lidarUpdateTaskHandle = nullptr;

// ==================== 函数实现 ====================

bool lidar_hardware_init() {
    // 初始化 RPLIDAR A1 驱动
    // 使用 Serial2 作为雷达串口
    if (!rplidar_init(&Serial2, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_MOTOR_PWM_PIN)) {
        Serial.println("[LIDAR] Hardware init failed");
        return false;
    }
    
    // 创建雷达数据处理任务
    if (!rplidar_create_task(8192, 10, 1)) {
        Serial.println("[LIDAR] Failed to create rplidar task");
        return false;
    }
    
    Serial.println("[LIDAR] Hardware initialized");
    return true;
}

bool lidar_start() {
    // 先停止雷达（如果之前在扫描）
    rplidar_stop_scan();
    rplidar_stop_motor();
    delay(100);
    
    // 启动电机
    rplidar_start_motor();
    delay(1000);  // 等待电机稳定
    
    // 获取设备信息（可选，失败不影响扫描）
    rplidar_response_device_info_t devInfo;
    if (rplidar_get_device_info(&devInfo)) {
        Serial.printf("[LIDAR] Model: %d, FW: %d.%02d, HW: %d\n",
                      devInfo.model,
                      devInfo.firmware_version >> 8,
                      devInfo.firmware_version & 0xFF,
                      devInfo.hardware_version);
        
        Serial.print("[LIDAR] S/N: ");
        for (int i = 0; i < 16; i++) {
            Serial.printf("%02X", devInfo.serialnum[i]);
        }
        Serial.println();
    } else {
        Serial.println("[LIDAR] Device info not available (continuing anyway)");
    }
    
    // 检查健康状态（可选）
    rplidar_response_device_health_t health;
    if (rplidar_get_health(&health)) {
        Serial.printf("[LIDAR] Health status: %d, error code: %d\n",
                      health.status, health.error_code);
        
        if (health.status == RPLIDAR_STATUS_ERROR) {
            Serial.println("[LIDAR] Warning: Lidar reports error state");
        }
    } else {
        Serial.println("[LIDAR] Health status not available (continuing anyway)");
    }
    
    // 启动扫描
    if (!rplidar_start_scan()) {
        Serial.println("[LIDAR] Failed to start scan");
        return false;
    }
    
    Serial.println("[LIDAR] Scanning started");
    return true;
}

void lidar_stop() {
    rplidar_stop_scan();
    rplidar_stop_motor();
    Serial.println("[LIDAR] Stopped");
}

bool lidar_publisher_init(rcl_node_t *node) {
    // 创建发布器
    if (rclc_publisher_init_default(
            &lidarPublisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            "/scan") != RCL_RET_OK) {
        Serial.println("[LIDAR] Failed to create LaserScan publisher");
        return false;
    }
    
    // 初始化消息
    memset(&lidarMsg, 0, sizeof(lidarMsg));
    
    // 设置 frame_id
    if (!rosidl_runtime_c__String__assign(&lidarMsg.header.frame_id, LIDAR_FRAME_ID)) {
        Serial.println("[LIDAR] Failed to set frame_id");
        return false;
    }
    
    // 设置扫描参数
    // RPLIDAR A1 参数：
    // - 角度范围: 0-360度
    // - 测量范围: 0.15m - 12m
    // - 扫描频率: ~5.5Hz (可变)
    lidarMsg.angle_min = 0.0f;                      // 起始角度 (rad)
    lidarMsg.angle_max = 2.0f * M_PI;               // 结束角度 (rad)
    lidarMsg.angle_increment = (2.0f * M_PI) / RPLIDAR_POINT_MAX;  // 角度增量
    lidarMsg.time_increment = 0.0f;                 // 时间增量（未使用）
    lidarMsg.scan_time = 0.1f;                      // 扫描时间（约10Hz）
    lidarMsg.range_min = LIDAR_RANGE_MIN;           // 最小测量距离
    lidarMsg.range_max = LIDAR_RANGE_MAX;           // 最大测量距离
    
    // 分配 ranges 和 intensities 数组
    lidarMsg.ranges.capacity = RPLIDAR_POINT_MAX;
    lidarMsg.ranges.size = RPLIDAR_POINT_MAX;
    lidarMsg.ranges.data = (float *)malloc(RPLIDAR_POINT_MAX * sizeof(float));
    if (lidarMsg.ranges.data == nullptr) {
        Serial.println("[LIDAR] Failed to allocate ranges array");
        return false;
    }
    
    lidarMsg.intensities.capacity = RPLIDAR_POINT_MAX;
    lidarMsg.intensities.size = RPLIDAR_POINT_MAX;
    lidarMsg.intensities.data = (float *)malloc(RPLIDAR_POINT_MAX * sizeof(float));
    if (lidarMsg.intensities.data == nullptr) {
        Serial.println("[LIDAR] Failed to allocate intensities array");
        free(lidarMsg.ranges.data);
        return false;
    }
    
    // 初始化数组
    for (int i = 0; i < RPLIDAR_POINT_MAX; i++) {
        lidarMsg.ranges.data[i] = 0.0f;
        lidarMsg.intensities.data[i] = 0.0f;
    }
    
    lidarPublisherReady = true;
    Serial.println("[LIDAR] Publisher initialized");
    return true;
}

void lidar_update_data() {
    if (!lidarPublisherReady) {
        return;
    }
    
    // 检查是否有新的扫描数据
    if (!rplidar_has_new_scan()) {
        return;
    }
    
    rplidar_clear_new_scan_flag();
    
    // 获取扫描数据
    rplidar_scan_data_t scanData;
    rplidar_get_scan_data(&scanData);
    
    if (!scanData.valid) {
        return;
    }
    
    // 更新消息数据
    // 将角度数据转换为 ROS LaserScan 格式
    // RPLIDAR 数据: 0度在正前方，顺时针
    // ROS LaserScan: 0度在正前方，逆时针（右手坐标系）
    for (int i = 0; i < RPLIDAR_POINT_MAX; i++) {
        // 角度映射：将 RPLIDAR 的顺时针转换为 ROS 的逆时针
        // RPLIDAR: 0->0, 90->270, 180->180, 270->90
        int rosIndex = (RPLIDAR_POINT_MAX - i) % RPLIDAR_POINT_MAX;
        
        float distance_mm = scanData.points[i].distance;
        float distance_m = distance_mm / 1000.0f;  // 转换为米
        
        // 过滤无效数据
        if (distance_m < LIDAR_RANGE_MIN || distance_m > LIDAR_RANGE_MAX || distance_mm == 0) {
            lidarMsg.ranges.data[rosIndex] = INFINITY;  // 无效数据用无穷大表示
        } else {
            lidarMsg.ranges.data[rosIndex] = distance_m;
        }
        
        // 信号强度（归一化到 0-1）
        lidarMsg.intensities.data[rosIndex] = (float)scanData.points[i].quality / 63.0f;
    }
}

void publishLidar() {
    if (!lidarPublisherReady) {
        return;
    }
    
    // 更新时间戳
    const uint32_t nowMs = millis();
    lidarMsg.header.stamp.sec = nowMs / 1000;
    lidarMsg.header.stamp.nanosec = (nowMs % 1000) * 1000000UL;
    
    // 发布消息
    rcl_publish(&lidarPublisher, &lidarMsg, NULL);
}

void lidar_update_task(void *arg) {
    Serial.printf("[LIDAR] Update task started on core %d\n", xPortGetCoreID());
    
    while (1) {
        // 更新激光雷达数据
        lidar_update_data();
        
        vTaskDelay(pdMS_TO_TICKS(LIDAR_UPDATE_INTERVAL_MS));
    }
    
    vTaskDelete(nullptr);
}

bool lidar_create_update_task(uint32_t stackSize, uint8_t priority, int coreId) {
    BaseType_t result;
    
    if (coreId >= 0) {
        result = xTaskCreatePinnedToCore(
            lidar_update_task,
            "lidar_update",
            stackSize,
            nullptr,
            priority,
            &lidarUpdateTaskHandle,
            coreId
        );
    } else {
        result = xTaskCreate(
            lidar_update_task,
            "lidar_update",
            stackSize,
            nullptr,
            priority,
            &lidarUpdateTaskHandle
        );
    }
    
    return (result == pdPASS);
}
