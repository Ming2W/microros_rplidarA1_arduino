/**
 * @file lidar_publisher.h
 * @brief 激光雷达 micro-ROS LaserScan 发布器头文件
 */

#ifndef LIDAR_PUBLISHER_H
#define LIDAR_PUBLISHER_H

#include <rcl/rcl.h>
#include <sensor_msgs/msg/laser_scan.h>

// ==================== 激光雷达发布器 ====================
extern rcl_publisher_t lidarPublisher;
extern sensor_msgs__msg__LaserScan lidarMsg;
extern bool lidarPublisherReady;

// ==================== 函数声明 ====================

/**
 * @brief 初始化激光雷达硬件（串口、电机）
 * @return true 初始化成功
 */
bool lidar_hardware_init();

/**
 * @brief 启动激光雷达扫描
 * @return true 启动成功
 */
bool lidar_start();

/**
 * @brief 停止激光雷达扫描
 */
void lidar_stop();

/**
 * @brief 初始化激光雷达 ROS 发布器
 * @param node ROS 节点指针
 * @return true 初始化成功
 */
bool lidar_publisher_init(rcl_node_t *node);

/**
 * @brief 更新激光雷达数据到 ROS 消息
 * 将最新的扫描数据填充到 LaserScan 消息中
 */
void lidar_update_data();

/**
 * @brief 发布激光雷达数据
 * 发布 sensor_msgs/LaserScan 消息到 /scan 话题
 */
void publishLidar();

/**
 * @brief 激光雷达数据更新任务（FreeRTOS 任务函数）
 * @param arg 任务参数
 */
void lidar_update_task(void *arg);

/**
 * @brief 创建激光雷达更新任务
 * @param stackSize 任务栈大小
 * @param priority 任务优先级
 * @param coreId 绑定的 CPU 核心 (-1 表示不绑定)
 * @return true 创建成功
 */
bool lidar_create_update_task(uint32_t stackSize = 8192, uint8_t priority = 5, int coreId = -1);

#endif // LIDAR_PUBLISHER_H
