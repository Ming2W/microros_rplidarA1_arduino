/**
 * @file rplidar_a1.cpp
 * @brief SLAMTEC RPLIDAR A1 驱动实现
 * 
 * 基于 SLAMTEC RPLIDAR 协议实现，用于 ESP32S3 Arduino 平台
 * 参考 MS200 驱动实现方式，使用状态机解析串口数据
 */

#include "rplidar_a1.h"

// ==================== 私有变量 ====================
static HardwareSerial *lidarSerial = nullptr;
static int motorPwmPin = -1;
static int motorPwmChannel = 0;

// 接收缓冲区
static uint8_t rxBuffer[RPLIDAR_RX_BUF_SIZE];
static uint16_t rxIndex = 0;

// 解析状态机
typedef enum {
    PARSE_IDLE = 0,
    PARSE_SYNC1,
    PARSE_SYNC2,
    PARSE_HEADER,
    PARSE_DATA,
    PARSE_MEASUREMENT
} parse_state_t;

static parse_state_t parseState = PARSE_IDLE;

// 当前扫描数据
static rplidar_scan_data_t currentScan = {0};
static rplidar_scan_data_t completedScan = {0};
static volatile bool newScanAvailable = false;

// 临时测量点缓存（用于组装一圈数据）
static rplidar_point_t tempPoints[RPLIDAR_POINT_MAX * 2];
static uint16_t tempPointCount = 0;

// 驱动状态
static rplidar_state_t driverState = RPLIDAR_STATE_IDLE;

// 响应解析
static rplidar_ans_header_t ansHeader;
static uint32_t expectedDataLen = 0;
static uint8_t expectedAnsType = 0;

// 任务句柄
static TaskHandle_t rplidarTaskHandle = nullptr;

// ==================== 私有函数声明 ====================
static void sendCommand(uint8_t cmd, const uint8_t *payload = nullptr, uint8_t payloadLen = 0);
static bool waitForResponse(uint8_t ansType, uint32_t timeoutMs = 1000);
static void parseResponseHeader();
static void parseMeasurementData(uint8_t *data);
static void processCompleteScan();

// ==================== 公共函数实现 ====================

bool rplidar_init(HardwareSerial *serial, int rxPin, int txPin, int pwmPin) {
    if (serial == nullptr) {
        return false;
    }
    
    lidarSerial = serial;
    motorPwmPin = pwmPin;
    
    // 初始化串口 - A1 使用 115200 波特率
    // 注意: Serial.begin(baud, config, rxPin, txPin)
    lidarSerial->begin(115200, SERIAL_8N1, rxPin, txPin);
    
    Serial.printf("[RPLIDAR] Serial init: RX=%d, TX=%d, Baud=115200\n", rxPin, txPin);
    
    // 初始化电机 PWM
    if (motorPwmPin >= 0) {
        // 使用 LEDC 产生 PWM 信号
        ledcSetup(motorPwmChannel, 25000, 8);  // 25kHz, 8位分辨率
        ledcAttachPin(motorPwmPin, motorPwmChannel);
        ledcWrite(motorPwmChannel, 0);  // 初始停止
        Serial.printf("[RPLIDAR] Motor PWM pin: %d\n", motorPwmPin);
    }
    
    // 清空缓冲区
    while (lidarSerial->available()) {
        lidarSerial->read();
    }
    
    // 初始化数据结构
    memset(&currentScan, 0, sizeof(currentScan));
    memset(&completedScan, 0, sizeof(completedScan));
    memset(tempPoints, 0, sizeof(tempPoints));
    tempPointCount = 0;
    
    parseState = PARSE_IDLE;
    driverState = RPLIDAR_STATE_IDLE;
    newScanAvailable = false;
    
    Serial.println("[RPLIDAR] Initialized");
    return true;
}

void rplidar_start_motor() {
    if (motorPwmPin >= 0) {
        ledcWrite(motorPwmChannel, 200);  // 约 78% 占空比
        Serial.println("[RPLIDAR] Motor started");
    }
}

void rplidar_stop_motor() {
    if (motorPwmPin >= 0) {
        ledcWrite(motorPwmChannel, 0);
        Serial.println("[RPLIDAR] Motor stopped");
    }
}

void rplidar_set_motor_pwm(uint8_t duty) {
    if (motorPwmPin >= 0) {
        ledcWrite(motorPwmChannel, duty);
    }
}

void rplidar_stop_scan() {
    sendCommand(RPLIDAR_CMD_STOP);
    driverState = RPLIDAR_STATE_IDLE;
    parseState = PARSE_IDLE;
    delay(10);
    
    // 清空接收缓冲区
    while (lidarSerial->available()) {
        lidarSerial->read();
    }
    
    Serial.println("[RPLIDAR] Scan stopped");
}

bool rplidar_start_scan() {
    // 先停止之前的扫描
    rplidar_stop_scan();
    delay(100);
    
    // 清空接收缓冲区
    while (lidarSerial->available()) {
        lidarSerial->read();
    }
    
    // 发送扫描命令
    Serial.println("[RPLIDAR] Sending SCAN command...");
    sendCommand(RPLIDAR_CMD_SCAN);
    
    // 等待响应头
    uint32_t startTime = millis();
    while (millis() - startTime < 2000) {
        int avail = lidarSerial->available();
        if (avail > 0) {
            Serial.printf("[RPLIDAR] Scan response: %d bytes available\n", avail);
            
            // 打印前几个字节用于调试
            if (avail >= 7) {
                uint8_t header[7];
                lidarSerial->readBytes(header, 7);
                
                Serial.printf("[RPLIDAR] Scan header: %02X %02X %02X %02X %02X %02X %02X\n",
                              header[0], header[1], header[2], header[3],
                              header[4], header[5], header[6]);
                
                if (header[0] == RPLIDAR_ANS_SYNC_BYTE1 && 
                    header[1] == RPLIDAR_ANS_SYNC_BYTE2 &&
                    header[6] == RPLIDAR_ANS_TYPE_MEASUREMENT) {
                    Serial.println("[RPLIDAR] Scan started successfully");
                    driverState = RPLIDAR_STATE_SCANNING;
                    parseState = PARSE_MEASUREMENT;
                    tempPointCount = 0;
                    return true;
                } else {
                    Serial.println("[RPLIDAR] Unexpected response header");
                }
            }
        }
        delay(10);
    }
    
    Serial.println("[RPLIDAR] Failed to start scan - timeout");
    return false;
}

void rplidar_reset() {
    sendCommand(RPLIDAR_CMD_RESET);
    delay(500);  // 等待雷达复位
    
    // 清空接收缓冲区
    while (lidarSerial->available()) {
        lidarSerial->read();
    }
    
    Serial.println("[RPLIDAR] Reset sent");
}

bool rplidar_get_device_info(rplidar_response_device_info_t *info) {
    if (info == nullptr) return false;
    
    // 清空接收缓冲区
    while (lidarSerial->available()) {
        lidarSerial->read();
    }
    
    Serial.println("[RPLIDAR] Sending GET_DEVICE_INFO command...");
    sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO);
    
    // 等待响应 - 增加超时时间到2秒
    uint32_t startTime = millis();
    while (millis() - startTime < 2000) {
        int avail = lidarSerial->available();
        if (avail > 0) {
            Serial.printf("[RPLIDAR] Received %d bytes\n", avail);
        }
        
        if (avail >= 7 + (int)sizeof(rplidar_response_device_info_t)) {
            // 读取响应头
            uint8_t header[7];
            lidarSerial->readBytes(header, 7);
            
            Serial.printf("[RPLIDAR] Header: %02X %02X %02X %02X %02X %02X %02X\n",
                          header[0], header[1], header[2], header[3],
                          header[4], header[5], header[6]);
            
            if (header[0] == RPLIDAR_ANS_SYNC_BYTE1 && 
                header[1] == RPLIDAR_ANS_SYNC_BYTE2 &&
                header[6] == RPLIDAR_ANS_TYPE_DEVINFO) {
                // 读取设备信息
                lidarSerial->readBytes((uint8_t*)info, sizeof(rplidar_response_device_info_t));
                return true;
            }
        }
        delay(10);
    }
    
    Serial.println("[RPLIDAR] GET_DEVICE_INFO timeout");
    return false;
}

bool rplidar_get_health(rplidar_response_device_health_t *health) {
    if (health == nullptr) return false;
    
    // 清空接收缓冲区
    while (lidarSerial->available()) {
        lidarSerial->read();
    }
    
    sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH);
    
    // 等待响应
    uint32_t startTime = millis();
    while (millis() - startTime < 1000) {
        if (lidarSerial->available() >= 7 + sizeof(rplidar_response_device_health_t)) {
            // 读取响应头
            uint8_t header[7];
            lidarSerial->readBytes(header, 7);
            
            if (header[0] == RPLIDAR_ANS_SYNC_BYTE1 && 
                header[1] == RPLIDAR_ANS_SYNC_BYTE2 &&
                header[6] == RPLIDAR_ANS_TYPE_DEVHEALTH) {
                // 读取健康状态
                lidarSerial->readBytes((uint8_t*)health, sizeof(rplidar_response_device_health_t));
                return true;
            }
        }
        delay(10);
    }
    
    return false;
}

void rplidar_process_data() {
    if (lidarSerial == nullptr || driverState != RPLIDAR_STATE_SCANNING) {
        return;
    }
    
    // 读取所有可用数据
    while (lidarSerial->available()) {
        uint8_t byte = lidarSerial->read();
        
        if (parseState == PARSE_MEASUREMENT) {
            // 直接解析测量数据（5字节一个点）
            rxBuffer[rxIndex++] = byte;
            
            if (rxIndex >= 5) {
                // 解析一个测量点
                parseMeasurementData(rxBuffer);
                rxIndex = 0;
            }
        }
    }
}

bool rplidar_has_new_scan() {
    return newScanAvailable;
}

void rplidar_clear_new_scan_flag() {
    newScanAvailable = false;
}

void rplidar_get_scan_data(rplidar_scan_data_t *data) {
    if (data != nullptr) {
        memcpy(data, &completedScan, sizeof(rplidar_scan_data_t));
    }
}

uint16_t rplidar_get_distance(uint16_t angle) {
    if (angle >= RPLIDAR_POINT_MAX) {
        return 0;
    }
    return (uint16_t)completedScan.points[angle].distance;
}

uint8_t rplidar_get_quality(uint16_t angle) {
    if (angle >= RPLIDAR_POINT_MAX) {
        return 0;
    }
    return completedScan.points[angle].quality;
}

rplidar_state_t rplidar_get_state() {
    return driverState;
}

void rplidar_task(void *arg) {
    Serial.printf("[RPLIDAR] Task started on core %d\n", xPortGetCoreID());
    
    while (1) {
        rplidar_process_data();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    vTaskDelete(nullptr);
}

bool rplidar_create_task(uint32_t stackSize, uint8_t priority, int coreId) {
    BaseType_t result;
    
    if (coreId >= 0) {
        result = xTaskCreatePinnedToCore(
            rplidar_task,
            "rplidar_task",
            stackSize,
            nullptr,
            priority,
            &rplidarTaskHandle,
            coreId
        );
    } else {
        result = xTaskCreate(
            rplidar_task,
            "rplidar_task",
            stackSize,
            nullptr,
            priority,
            &rplidarTaskHandle
        );
    }
    
    return (result == pdPASS);
}

// ==================== 私有函数实现 ====================

static void sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen) {
    if (lidarSerial == nullptr) return;
    
    // 发送命令
    lidarSerial->write(RPLIDAR_CMD_SYNC_BYTE);
    
    if (payload != nullptr && payloadLen > 0) {
        lidarSerial->write(cmd | RPLIDAR_CMDFLAG_HAS_PAYLOAD);
        lidarSerial->write(payloadLen);
        
        // 计算校验和
        uint8_t checksum = 0 ^ RPLIDAR_CMD_SYNC_BYTE ^ (cmd | RPLIDAR_CMDFLAG_HAS_PAYLOAD) ^ payloadLen;
        for (uint8_t i = 0; i < payloadLen; i++) {
            lidarSerial->write(payload[i]);
            checksum ^= payload[i];
        }
        lidarSerial->write(checksum);
    } else {
        lidarSerial->write(cmd);
    }
    
    lidarSerial->flush();
}

static void parseMeasurementData(uint8_t *data) {
    rplidar_response_measurement_node_t node;
    memcpy(&node, data, 5);
    
    // 检查校验位
    uint8_t checkBit = node.angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
    uint8_t syncBit = node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
    uint8_t syncBitInverse = (node.sync_quality >> 1) & 0x01;
    
    // 校验位应该是 1，且 sync 和 sync_inverse 应该相反
    if (checkBit != 1 || syncBit == syncBitInverse) {
        // 数据校验失败，跳过
        return;
    }
    
    // 解析数据
    rplidar_point_t point;
    point.startBit = (syncBit == 1);
    point.quality = (node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    point.angle = (float)(node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
    point.distance = (float)node.distance_q2 / 4.0f;  // 转换为 mm
    
    // 如果是新一圈的起点，处理之前的数据
    if (point.startBit && tempPointCount > 0) {
        processCompleteScan();
    }
    
    // 添加到临时缓存
    if (tempPointCount < RPLIDAR_POINT_MAX * 2) {
        tempPoints[tempPointCount++] = point;
    }
}

static void processCompleteScan() {
    if (tempPointCount == 0) return;
    
    // 清空完成的扫描数据
    memset(&currentScan, 0, sizeof(currentScan));
    
    // 将临时点转换到固定角度数组
    for (uint16_t i = 0; i < tempPointCount; i++) {
        uint16_t angleIndex = (uint16_t)tempPoints[i].angle;
        if (angleIndex >= RPLIDAR_POINT_MAX) {
            angleIndex = angleIndex % RPLIDAR_POINT_MAX;
        }
        
        // 使用距离更近的点，或者信号更强的点
        if (currentScan.points[angleIndex].distance == 0 ||
            (tempPoints[i].quality > currentScan.points[angleIndex].quality)) {
            currentScan.points[angleIndex] = tempPoints[i];
        }
    }
    
    // 计算有效点数
    currentScan.count = 0;
    for (uint16_t i = 0; i < RPLIDAR_POINT_MAX; i++) {
        if (currentScan.points[i].distance > 0) {
            currentScan.count++;
        }
    }
    
    currentScan.valid = (currentScan.count > 50);  // 至少 50 个有效点
    currentScan.timestamp = millis();
    
    // 复制到完成的扫描数据
    memcpy(&completedScan, &currentScan, sizeof(rplidar_scan_data_t));
    newScanAvailable = true;
    
    // 清空临时缓存
    tempPointCount = 0;
}
