#include "app.hpp"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "eskf.h"
#include "eskf_core.h"
#include "time_sync.h"
#include <cstddef>
#include <cstdio>

// --- printf 重定向支持 ---
// 使用 HAL 库的 UART 句柄发送 (需要在 main.c 中定义 huart1)
#include "usart.h" 

// 如果你的编译器是 GCC (STM32CubeIDE)
extern "C" int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, 100);
    return len;
}


// -----------------------

eskf_t eskf; // ESKF 实例
time_sync_t lidar_time_sync; // Lidar 时间同步器实例

osMutexId_t eskf_mutex_handle; // ESKF 互斥锁句柄
osMessageQueueId_t imu_data_queue_handle; // IMU 数据队列句柄
osMessageQueueId_t odom_data_queue_handle; // Odom 数据队列句柄

Lidar_data_t lidar_data_buffer; // Lidar 数据缓冲区

// imu数据获取中断处理函数
void IMU_get_interrupt_handler(void *argument) {
    Imu_data_t imu_data_temp; // IMU 数据缓冲区
    IMU_get_data(&imu_data_temp); // 获取 IMU 数据并存储到缓冲区

    // 使用系统时间作为时间戳，确保所有传感器在同一时间基准
    imu_data_temp.t = get_current_time();
    
    osMessageQueuePut(imu_data_queue_handle, &imu_data_temp, 0, 5); // 将 IMU 数据放入队列
}

// 里程计数据获取中断处理函数
void Odom_get_interrupt_handler(void *argument) {
    Odom_data_t odom_data_temp;
    Odom_get_data(&odom_data_temp); // 获取里程计数据并存储到缓冲区
    // odom_data_temp.update_flag = 1; // 队列本身就代表了更新，不再需要 flag
    osMessageQueuePut(odom_data_queue_handle, &odom_data_temp, 0, 0);
}


extern "C" void Init(void *argument) {
    // 初始化 ESKF 应用
    ESKF_App_Init(&eskf);
    // 创建互斥锁
    eskf_mutex_handle = osMutexNew(NULL);
    // 创建IMU储存队列
    imu_data_queue_handle = osMessageQueueNew(20, sizeof(Imu_data_t), NULL); // 队列长度20，元素大小为Imu_data_t
    // 创建Odom储存队列
    odom_data_queue_handle = osMessageQueueNew(10, sizeof(Odom_data_t), NULL);

    // 初始化 Lidar 时间同步器
    time_sync_init(&lidar_time_sync, 0.1f, 0.001f);
    
    osThreadExit();
}




// 注意，imu_odom使用的时间戳是系统时间戳，而Lidar使用的时间戳是经过time_sync对齐后的时间戳，确保两者在同一时间基准下工作。

extern "C" void Imu_Odom_Task(void *argument) {
    Imu_data_t imu_data_buffer; // IMU 数据缓冲区
    Odom_data_t odom_data_buffer; // Odom 数据缓冲区

    float t_last = -1.0f; // 上一次 IMU 数据的时间戳
    for(;;){
        if (osMessageQueueGet(imu_data_queue_handle, &imu_data_buffer, NULL, osWaitForever) == osOK) {
            osMutexAcquire(eskf_mutex_handle, osWaitForever); // 获取互斥锁，保护 ESKF 数据结构
            // 计算时间间隔
            
            if (t_last < 0.0f) {
                t_last = imu_data_buffer.t; // 初始化时间戳
                osMutexRelease(eskf_mutex_handle); // 释放互斥锁
                continue; // 跳过第一次推算，等待下一次 IMU 数据到达
            }
            float dt = imu_data_buffer.t - t_last;
            
            // [修改] 增加时间异常保护
            // 1. 防止 dt <= 0 (可能由于时间戳精度或中断处理顺序)
            // 2. 防止 dt 过大 (如丢包或调试暂停后恢复)，避免积分发散
            const float MAX_DT = 0.2f; // 允许的最大间隔，例如 5Hz
            if (dt <= 0.00001f || dt > MAX_DT) {
                t_last = imu_data_buffer.t;
                osMutexRelease(eskf_mutex_handle);
                continue;
            }

            t_last = imu_data_buffer.t;
            ESKF_App_IMU_Handler(&eskf, imu_data_buffer.acc,
                                imu_data_buffer.gyro,
                                         dt, imu_data_buffer.t);
            // 处理里程计数据（如果有更新）
            // 非阻塞检查 Odom 数据
            while (osMessageQueueGet(odom_data_queue_handle, &odom_data_buffer, NULL, 0) == osOK) {
                ESKF_App_Odom_Handler(&eskf, odom_data_buffer.v_body_x,
                                        odom_data_buffer.v_body_y,
                                        odom_data_buffer.v_body_wz);
            }
            osMutexRelease(eskf_mutex_handle); // 释放互斥锁
        }
    }
}

extern "C" void Lidar_Task(void *argument) {
    for(;;){
        Lidar_get_data(&lidar_data_buffer); // 获取 Lidar 数据并存储到缓冲区
        
        // --- 使用 time_sync 模块进行对齐 ---
       
        float sys_time_now = get_current_time();
        
        // 更新同步器状态
        // 输入: (PC发送时间/Lidar内部时间, STM32接收时间)
        time_sync_update(&lidar_time_sync, lidar_data_buffer.start_t, sys_time_now);
        
        // 获取对齐后的系统时间
        // aligned_time = lidar_data.start_t + offset
        float lidar_sys_time = (float)time_sync_get_aligned_time(&lidar_time_sync, lidar_data_buffer.start_t);
    
        if (osMutexAcquire(eskf_mutex_handle, 100) == osOK) { // 尝试获取互斥锁，避免长时间阻塞
             ESKF_App_Lidar_Handler(&eskf, lidar_data_buffer.pos_x,
                                lidar_data_buffer.pos_y,
                                lidar_data_buffer.yaw,
                                1,
                                lidar_sys_time); // [修改] 传递对齐后的时间戳
            osMutexRelease(eskf_mutex_handle); // 释放互斥锁
        }
        osDelay(10);
    }
    
}

extern "C" void Output_Task(void *argument){
    for(;;){
        if (osMutexAcquire(eskf_mutex_handle, 100) == osOK) { // 尝试获取互斥锁，避免长时间阻塞
            eskf_state_t current_state;
            ESKF_App_Get_State(&eskf, &current_state); // 获取当前状态
            osMutexRelease(eskf_mutex_handle); // 释放互斥锁
            // 输出状态 (例如：通过串口发送，或更新显示屏)
            // 输出格式示例: "POS: (x, y)"
            printf("POS: (%.2f, %.2f)\n", current_state.pos.x, current_state.pos.y);
        }
        osDelay(100); // 每 100ms 输出一次状态

    }
}

// --- Dummy Implementations (Added to fix linker errors) ---
float get_current_time() {
    return 1.0f; 
}

void IMU_get_data(Imu_data_t *imu_data) {
    }

void Lidar_get_data(Lidar_data_t *lidar_data) {
    }

void Odom_get_data(Odom_data_t *odom_data) {
    }








