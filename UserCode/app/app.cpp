#include "app.hpp"


// --- printf 重定向支持 ---
// 使用 HAL 库的 UART 句柄发送 (需要在 main.c 中定义 huart1)
#include "usart.h" 

// 如果你的编译器是 GCC (STM32CubeIDE)
extern "C" int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, 100);
    return len;
}

// -----------------------

// ESKF 定位系统实例 (使用 C++ 封装类)
chassis_loc::LocESKF loc_eskf;
time_sync_t lidar_time_sync; // Lidar 时间同步器实例

osMutexId_t eskf_mutex_handle; // ESKF 互斥锁句柄

osMessageQueueId_t imu_data_queue_handle; // IMU 数据队列句柄
osMessageQueueId_t odom_data_queue_handle; // Odom 数据队列句柄
osSemaphoreId_t lidar_data_sem_handle; // Lidar 数据信号量句柄
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
    
    // 使用系统时间作为时间戳，确保与 IMU 数据在同一时间基准
    odom_data_temp.t = get_current_time();
    
    osMessageQueuePut(odom_data_queue_handle, &odom_data_temp, 0, 0);
}

// Lidar数据获取中断处理函数
void Lidar_get_interrupt_handler(void *argument) {
    Lidar_get_data(&lidar_data_buffer); // 获取 Lidar 数据并存储到缓冲区
    osSemaphoreRelease(lidar_data_sem_handle); // 释放信号量，通知 Lidar_Task 数据已就绪
}


extern "C" void Init(void *argument) {
    // 初始化 ESKF 定位系统 (使用默认配置)
    if (!loc_eskf.Init()) {
        // 初始化失败处理
        printf("ESKF Init Failed!\n");
    }
    
    // 创建互斥锁
    eskf_mutex_handle = osMutexNew(NULL);
    // 创建IMU储存队列
    imu_data_queue_handle = osMessageQueueNew(20, sizeof(Imu_data_t), NULL); // 队列长度20，元素大小为Imu_data_t
    // 创建Odom储存队列
    odom_data_queue_handle = osMessageQueueNew(10, sizeof(Odom_data_t), NULL);
    // 创建Lidar信号量
    lidar_data_sem_handle = osSemaphoreNew(0, 1, NULL);
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
            
            // 使用 LocESKF 处理 IMU 数据
            loc_eskf.UpdateIMU(imu_data_buffer.acc,
                             imu_data_buffer.gyro,
                             dt, 
                             imu_data_buffer.t);
            
            // 处理里程计数据（如果有更新）
            // 非阻塞检查 Odom 数据
            while (osMessageQueueGet(odom_data_queue_handle, &odom_data_buffer, NULL, 0) == osOK) {
                // 使用里程计数据的时间戳，确保与 IMU 数据在同一时间基准
                loc_eskf.UpdateOdom(odom_data_buffer.v_body_x,
                                  odom_data_buffer.v_body_y,
                                  odom_data_buffer.v_body_wz,
                                  odom_data_buffer.t);
            }
            osMutexRelease(eskf_mutex_handle); // 释放互斥锁
        }
    }
}

extern "C" void Lidar_Task(void *argument) {
    for(;;){
        // 等待信号量（阻塞，直到 Lidar 数据到达）
        if (osSemaphoreAcquire(lidar_data_sem_handle, osWaitForever) == osOK) {
            // 数据已经在中断中读取到 lidar_data_buffer
            
            // --- 使用 time_sync 模块进行对齐 ---
           
            float sys_time_now = get_current_time();
            
            // 更新同步器状态
            // 输入: (上位机发送时间send_t, STM32接收时间sys_time_now)
            // 使用 send_t 计算时钟偏移（因为 send_t 是上位机实际发送的时刻）
            time_sync_update(&lidar_time_sync, lidar_data_buffer.send_t, sys_time_now);
            
            // 获取对齐后的系统时间
            // 使用 start_t（lidar开始检测的时间）转换为STM32时间轴
            // aligned_time = start_t + offset
            // 这个时间用于滤波器状态回退
            float lidar_sys_time = (float)time_sync_get_aligned_time(&lidar_time_sync, lidar_data_buffer.start_t);
        
            if (osMutexAcquire(eskf_mutex_handle, 100) == osOK) {
                // 使用 LocESKF 处理 Lidar 数据
                loc_eskf.UpdateLidar(lidar_data_buffer.pos_x,
                                   lidar_data_buffer.pos_y,
                                   lidar_data_buffer.yaw,
                                   1,  // has_yaw = 1
                                   lidar_sys_time);
                osMutexRelease(eskf_mutex_handle);
            }
        }
    }
}

extern "C" void Output_Task(void *argument){
    for(;;){
        if (osMutexAcquire(eskf_mutex_handle, 100) == osOK) { // 尝试获取互斥锁，避免长时间阻塞
            // 通过 LocESKF 获取当前状态
            const auto& posture = loc_eskf.posture().in_world;
            const auto& velocity = loc_eskf.velocity().in_world;
            
            osMutexRelease(eskf_mutex_handle); // 释放互斥锁
            
            // 输出状态 (通过串口发送)
            // 输出格式: "POS: (x, y, yaw) VEL: (vx, vy, wz)"
            printf("POS: (%.2f, %.2f, %.2f) VEL: (%.2f, %.2f, %.2f)\n", 
                   posture.x, posture.y, posture.yaw,
                   velocity.vx, velocity.vy, velocity.wz);
        }
        osDelay(100); // 每 100ms 输出一次状态
    }
}

// --- Dummy Implementations (Added to fix linker errors) ---
float get_current_time() {
    return 0.0f; 
}

void IMU_get_data(Imu_data_t *imu_data) {
}

void Lidar_get_data(Lidar_data_t *lidar_data) {
}

void Odom_get_data(Odom_data_t *odom_data) {
}
