/**
 * @file main.cpp
 * @brief RPLIDAR A1 激光雷达调试程序
 * 
 * 简化版本，仅包含雷达相关功能，用于调试
 */

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "config.h"
#include "pins.h"
#include "lidar_publisher.h"
#include "rplidar_a1.h"

// ==================== micro-ROS 全局变量 ====================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static bool microrosReady = false;

// ==================== 时间控制 ====================
uint32_t lastLidarPublishMs = 0;
uint32_t lastStatusPrintMs = 0;

/**
 * @brief 初始化 micro-ROS
 */
bool initMicroROS() {
    IPAddress agentIp;
    agentIp.fromString(AGENT_IP);
    
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, agentIp, AGENT_PORT);
    delay(2000);

    allocator = rcl_get_default_allocator();
    
    Serial.printf("Connecting to micro-ROS agent: %s:%d\n", AGENT_IP, AGENT_PORT);
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        Serial.println("micro-ROS support init failed");
        return false;
    }

    if (rclc_node_init_default(&node, "lidar_node", "", &support) != RCL_RET_OK) {
        Serial.println("micro-ROS node init failed");
        return false;
    }

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        Serial.println("Executor init failed");
        return false;
    }

    Serial.println("micro-ROS initialized successfully");
    return true;
}

/**
 * @brief 初始化硬件和 micro-ROS
 */
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  RPLIDAR A1 Lidar Debug Program");
    Serial.println("========================================\n");

    // 初始化 micro-ROS
    if (!initMicroROS()) {
        Serial.println("ERROR: micro-ROS init failed!");
        while (1) { delay(1000); }
    }
    
    // 初始化激光雷达发布器
    if (!lidar_publisher_init(&node)) {
        Serial.println("ERROR: Lidar publisher init failed!");
        while (1) { delay(1000); }
    }

    // 初始化激光雷达硬件
    if (!lidar_hardware_init()) {
        Serial.println("ERROR: Lidar hardware init failed!");
        while (1) { delay(1000); }
    }
    
    // 创建雷达数据更新任务（增大栈大小防止溢出）
    if (!lidar_create_update_task(8192, 5, 1)) {
        Serial.println("ERROR: Failed to create lidar update task!");
        while (1) { delay(1000); }
    }

    // 启动雷达扫描
    if (!lidar_start()) {
        Serial.println("ERROR: Failed to start lidar!");
        while (1) { delay(1000); }
    }

    microrosReady = true;
    lastLidarPublishMs = millis();
    lastStatusPrintMs = millis();
    
    Serial.println("\n[OK] All initialized! Starting main loop...\n");
}

/**
 * @brief 主循环
 */
void loop() {
    const uint32_t now = millis();

    // 处理 micro-ROS 事件
    if (microrosReady) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }

    // 发布激光雷达数据
    if (now - lastLidarPublishMs >= LIDAR_PUBLISH_INTERVAL_MS) {
        lastLidarPublishMs = now;
        publishLidar();
    }

    // 每2秒打印一次状态信息
    if (now - lastStatusPrintMs >= 2000) {
        lastStatusPrintMs = now;
        
        rplidar_state_t state = rplidar_get_state();
        Serial.printf("[STATUS] Lidar state: %s, New scan: %s\n",
                      state == RPLIDAR_STATE_SCANNING ? "SCANNING" : 
                      state == RPLIDAR_STATE_IDLE ? "IDLE" : "OTHER",
                      rplidar_has_new_scan() ? "YES" : "NO");
        
        // 打印几个角度的距离值作为参考
        Serial.printf("[DATA] Distance at 0°: %dmm, 90°: %dmm, 180°: %dmm, 270°: %dmm\n",
                      rplidar_get_distance(0),
                      rplidar_get_distance(90),
                      rplidar_get_distance(180),
                      rplidar_get_distance(270));
    }

    delay(5);
}
