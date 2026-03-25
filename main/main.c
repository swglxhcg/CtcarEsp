#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "math.h"

#include "uart0.h"
#include "uart1.h"
#include "lidar_ms200.h"
#include "icm42670p.h"

static const char *TAG = "CtESP_MAIN";

#define POINTS_COUNT        360
#define RX_BUFFER_SIZE      256     // 接收缓冲区大小
#define MAX_PAYLOAD_SIZE    800     // 最大负载大小

// --- 协议帧头与类型 ---
#define HEADER_1            0xAA
#define HEADER_SYNC         0x50  
#define HEADER_LIDAR        0x55  
#define HEADER_IMU          0x56  

#define TYPE_SYNC           0x00
#define TYPE_LIDAR          0x01
#define TYPE_IMU            0x02
#define TYPE_COMMAND_ACK    0x03  // 新增：对上位机指令的应答

// 指令定义 (由上位机发送)
#define CMD_STOP_LIDAR      0x10
#define CMD_START_LIDAR     0x11
#define CMD_STOP_IMU        0x20
#define CMD_START_IMU       0x21
#define CMD_REQ_SYNC        0x30

#define FOOTER_1            0x0D
#define FOOTER_2            0x0A

// --- 全局状态标志 ---
static volatile bool g_lidar_enabled = true;
static volatile bool g_imu_enabled = true;

// --- 通用发送函数 (使用 Uart0_Send_Data) ---
void send_frame(uint8_t type_id, uint8_t *payload, uint16_t payload_len, uint32_t timestamp_ms) {
    uint8_t tx_buffer[MAX_PAYLOAD_SIZE + 20]; // 留有余量
    int idx = 0;
    uint8_t checksum = 0;

    // 确定帧头第二个字节
    uint8_t header2 = 0;
    if (type_id == TYPE_SYNC) header2 = HEADER_SYNC;
    else if (type_id == TYPE_LIDAR) header2 = HEADER_LIDAR;
    else if (type_id == TYPE_IMU) header2 = HEADER_IMU;
    else if (type_id == TYPE_COMMAND_ACK) header2 = 0x57; // 自定义应答帧头
    else return; // 未知类型
    
    // 1. 帧头
    tx_buffer[idx++] = HEADER_1;
    tx_buffer[idx++] = header2;
    checksum += HEADER_1; 
    checksum += header2;

    // 2. 总数据长度 = Type(1) + Payload(N) + Timestamp(4)
    uint16_t total_data_len = 1 + payload_len + 4;
    tx_buffer[idx++] = (uint8_t)(total_data_len & 0xFF);
    tx_buffer[idx++] = (uint8_t)((total_data_len >> 8) & 0xFF);
    checksum += tx_buffer[idx-2]; 
    checksum += tx_buffer[idx-1];

    // 3. 类型 ID
    tx_buffer[idx++] = type_id;
    checksum += type_id;

    // 4. 负载数据
    if (payload_len > 0 && payload != NULL) {
        memcpy(&tx_buffer[idx], payload, payload_len);
        for(int i=0; i<payload_len; i++) {
            checksum += tx_buffer[idx+i];
        }
        idx += payload_len;
    }

    // 5. 时间戳 (uint32_t, ms) - 注意端序，这里假设小端
    memcpy(&tx_buffer[idx], &timestamp_ms, 4);
    for(int i=0; i<4; i++) {
        checksum += tx_buffer[idx+i];
    }
    idx += 4;

    // 6. 校验和
    tx_buffer[idx++] = checksum;

    // 7. 帧尾
    tx_buffer[idx++] = FOOTER_1;
    tx_buffer[idx++] = FOOTER_2;

    // 调用提供的发送函数
    if (Uart0_Send_Data(tx_buffer, idx) != idx) {
        ESP_LOGW(TAG, "UART Send incomplete or failed");
    }
}

// --- 指令处理任务 ---
void command_rx_task(void *arg) {
    ESP_LOGI(TAG, "Command Receiver Task Started");
    
    uint8_t rx_buf[RX_BUFFER_SIZE];
    int buf_idx = 0;
    
    // 解析状态机变量
    enum {
        STATE_WAIT_HEADER1,
        STATE_WAIT_HEADER2,
        STATE_WAIT_LEN_LOW,
        STATE_WAIT_LEN_HIGH,
        STATE_WAIT_TYPE,
        STATE_WAIT_PAYLOAD,
        STATE_WAIT_TIMESTAMP,
        STATE_WAIT_CHECKSUM,
        STATE_WAIT_FOOTER1,
        STATE_WAIT_FOOTER2
    } state = STATE_WAIT_HEADER1;

    uint8_t expected_header2 = 0;
    uint16_t total_data_len = 0;
    uint16_t payload_len = 0;
    uint8_t cmd_type = 0;
    uint8_t calculated_checksum = 0;
    uint8_t current_checksum = 0;
    uint16_t bytes_read = 0;
    uint8_t temp_byte = 0;
    uint32_t timestamp_temp = 0; // 仅用于跳过时间戳字节

    while(1) {
        // 检查是否有数据
        if (Uart0_Available() > 0) {
            temp_byte = Uart0_Read();
            
            switch (state) {
                case STATE_WAIT_HEADER1:
                    if (temp_byte == HEADER_1) {
                        calculated_checksum = HEADER_1;
                        state = STATE_WAIT_HEADER2;
                    }
                    break;

                case STATE_WAIT_HEADER2:
                    // 这里我们只关心特定的指令帧头，假设上位机发送指令使用特殊的第二个字节
                    // 例如定义 0xA0 为指令帧头，或者复用现有帧头但通过Type区分
                    // 为了通用性，这里假设只要是 0xAA 开头的都尝试解析，通过 Type 判断是否是指令
                    // 或者你可以定义一个新的 HEADER_CMD 0xA0
                    // 此处演示：如果收到 0xAA 0xA0 视为指令帧，其他视为普通数据帧（本例主要处理指令）
                    // 简化处理：假设上位机发送指令格式与普通数据帧一致，只是 Type 不同
                    
                    calculated_checksum += temp_byte;
                    
                    // 简单策略：接受任何第二个字节，后续通过 Type 判断
                    // 如果需要严格过滤，可在此加 if (temp_byte != 0xA0) { state = STATE_WAIT_HEADER1; break; }
                    
                    state = STATE_WAIT_LEN_LOW;
                    break;

                case STATE_WAIT_LEN_LOW:
                    calculated_checksum += temp_byte;
                    total_data_len = temp_byte;
                    state = STATE_WAIT_LEN_HIGH;
                    break;

                case STATE_WAIT_LEN_HIGH:
                    calculated_checksum += temp_byte;
                    total_data_len |= (temp_byte << 8);
                    
                    // 校验长度合理性 (最小长度：Type(1)+Time(4)=5)
                    if (total_data_len < 5 || total_data_len > (RX_BUFFER_SIZE - 10)) {
                        ESP_LOGW(TAG, "Invalid length: %d, resetting", total_data_len);
                        state = STATE_WAIT_HEADER1;
                        break;
                    }
                    
                    payload_len = total_data_len - 5; // 减去 Type(1) 和 Time(4)
                    bytes_read = 0;
                    state = STATE_WAIT_TYPE;
                    break;

                case STATE_WAIT_TYPE:
                    calculated_checksum += temp_byte;
                    cmd_type = temp_byte;
                    
                    if (payload_len > 0) {
                        buf_idx = 0;
                        state = STATE_WAIT_PAYLOAD;
                    } else {
                        // 没有负载，直接去读时间戳
                        bytes_read = 0;
                        state = STATE_WAIT_TIMESTAMP;
                    }
                    break;

                case STATE_WAIT_PAYLOAD:
                    calculated_checksum += temp_byte;
                    if (buf_idx < RX_BUFFER_SIZE) {
                        rx_buf[buf_idx++] = temp_byte;
                    }
                    bytes_read++;
                    if (bytes_read >= payload_len) {
                        bytes_read = 0;
                        state = STATE_WAIT_TIMESTAMP;
                    }
                    break;

                case STATE_WAIT_TIMESTAMP:
                    calculated_checksum += temp_byte;
                    bytes_read++;
                    if (bytes_read >= 4) {
                        bytes_read = 0;
                        state = STATE_WAIT_CHECKSUM;
                    }
                    break;

                case STATE_WAIT_CHECKSUM:
                    current_checksum = temp_byte;
                    state = STATE_WAIT_FOOTER1;
                    break;

                case STATE_WAIT_FOOTER1:
                    if (temp_byte == FOOTER_1) {
                        state = STATE_WAIT_FOOTER2;
                    } else {
                        ESP_LOGW(TAG, "Footer1 mismatch, reset");
                        state = STATE_WAIT_HEADER1;
                    }
                    break;

                case STATE_WAIT_FOOTER2:
                    if (temp_byte == FOOTER_2) {
                        // 帧完整，验证校验和
                        if (current_checksum == calculated_checksum) {
                            ESP_LOGI(TAG, "Valid Command Frame Received. Type: 0x%02X", cmd_type);
                            
                            // --- 执行指令逻辑 ---
                            uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000);
                            uint8_t ack_payload[2] = {cmd_type, 0x01}; // 0x01 表示成功

                            switch (cmd_type) {
                                case CMD_STOP_LIDAR:
                                    g_lidar_enabled = false;
                                    ESP_LOGI(TAG, "LiDAR Streaming Stopped");
                                    send_frame(TYPE_COMMAND_ACK, ack_payload, 2, ts);
                                    break;
                                    
                                case CMD_START_LIDAR:
                                    g_lidar_enabled = true;
                                    ESP_LOGI(TAG, "LiDAR Streaming Started");
                                    send_frame(TYPE_COMMAND_ACK, ack_payload, 2, ts);
                                    break;

                                case CMD_STOP_IMU:
                                    g_imu_enabled = false;
                                    ESP_LOGI(TAG, "IMU Streaming Stopped");
                                    send_frame(TYPE_COMMAND_ACK, ack_payload, 2, ts);
                                    break;

                                case CMD_START_IMU:
                                    g_imu_enabled = true;
                                    ESP_LOGI(TAG, "IMU Streaming Started");
                                    send_frame(TYPE_COMMAND_ACK, ack_payload, 2, ts);
                                    break;

                                case CMD_REQ_SYNC:
                                    ESP_LOGI(TAG, "Sync Requested by Host");
                                    send_frame(TYPE_SYNC, NULL, 0, ts);
                                    break;

                                default:
                                    ESP_LOGW(TAG, "Unknown Command: 0x%02X", cmd_type);
                                    // 可以选择发送一个错误应答
                                    break;
                            }
                        } else {
                            ESP_LOGW(TAG, "Checksum Mismatch! Calc: 0x%02X, Recv: 0x%02X", calculated_checksum, current_checksum);
                        }
                    } else {
                        ESP_LOGW(TAG, "Footer2 mismatch, reset");
                    }
                    // 无论成功失败，重置状态
                    state = STATE_WAIT_HEADER1;
                    break;
                
                default:
                    state = STATE_WAIT_HEADER1;
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 短暂延时，避免空转占用过多CPU
    }
}

// --- LiDAR 发送任务 ---
void lidar_task(void *arg) {
    ESP_LOGI(TAG, "LiDAR Task Started (10Hz)");
    uint8_t dist_buffer[POINTS_COUNT * 2];
    
    while(1) {
        if (g_lidar_enabled) {
            // 采集
            for(int i=0; i<POINTS_COUNT; i++) {
                uint16_t d = Lidar_Ms200_Get_Distance(i);
                dist_buffer[i*2] = d & 0xFF;
                dist_buffer[i*2+1] = (d >> 8) & 0xFF;
            }
            
            uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000);
            send_frame(TYPE_LIDAR, dist_buffer, POINTS_COUNT * 2, ts);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

// --- IMU 发送任务 ---
void imu_task(void *arg) {
    ESP_LOGI(TAG, "IMU Task Started (100Hz)");
    float data[6]; 
    
    while(1) {
        if (g_imu_enabled) {
            float accel[3], gyro[3];
            Icm42670p_Get_Accel_g(accel);
            Icm42670p_Get_Gyro_dps(gyro);
            
            memcpy(data, accel, 12);
            memcpy(data+3, gyro, 12);
            
            uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000);
            send_frame(TYPE_IMU, (uint8_t*)data, 24, ts);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// --- 初始同步任务 (可选，启动时发一次) ---
void sync_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Sending Initial Time Sync Frame...");
    uint32_t current_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    send_frame(TYPE_SYNC, NULL, 0, current_time_ms);
    vTaskDelete(NULL);
}

void Component_Init(void){
    Uart0_Init();
    Uart1_Init();
    Lidar_Ms200_Init();
    Icm42670p_Init();
}

void app_main(void) {
    ESP_LOGI(TAG, "System Started. Initializing UART0 Bridge...");
    Component_Init();

    // 创建任务
    xTaskCreate(sync_task, "sync_task", 2048, NULL, 5, NULL);
    xTaskCreate(lidar_task, "lidar_task", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 3072, NULL, 6, NULL);
    
    // 新增：指令接收任务 (优先级稍高，以便及时响应)
    xTaskCreate(command_rx_task, "cmd_rx_task", 4096, NULL, 7, NULL);

    ESP_LOGI(TAG, "All tasks created. Waiting for commands...");
}