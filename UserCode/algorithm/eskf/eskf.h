#ifndef ESKF_H
#define ESKF_H

#include "eskf_types.h"
#include "eskf_core.h"
#include "eskf_math.h"

#ifdef __cplusplus
extern "C" {
#endif

// 常量定义
#define CONST_G          9.81f          // 重力加速度 (m/s^2)
#define DEG2RAD_F        0.0174532925f  // 度转弧度系数 (float)
#define RAD2DEG_F        57.2957795f    // 弧度转度系数 (float)
// -------------------------------------------------------------
// 应用层接口 - 用于在主循环或RTOS任务中调用
// -------------------------------------------------------------

/**
 * @brief 系统初始化，配置 ESKF 参数并重置状态
 * @param eskf ESKF 实例指针
 */
void ESKF_App_Init(eskf_t *eskf);

/**
 * @brief IMU 数据处理回调函数
 *        建议在 100Hz ~ 400Hz 的定时器中断或高优先级任务中调用
 * 
 * @param eskf     ESKF 实例指针
 * @param acc_raw  三轴加速度原始数据 (单位: g, e.g. 1.0 = 9.8m/s^2)
 * @param gyro_raw 三轴角速度原始数据 (单位: dps, degree per second)
 * @param dt       距离上一次调用的时间间隔 (单位: s)
 * @param timestamp 数据采集时刻的系统时间戳 (s)
 */
void ESKF_App_IMU_Handler(eskf_t *eskf, float acc_raw[3], float gyro_raw[3], float dt, double timestamp);

/**
 * @brief 轮速计/编码器 速度数据处理回调函数
 *        可用于高频更新 (20Hz ~ 100Hz)
 * 
 * @param eskf       ESKF 实例指针
 * @param v_body_x   车体系前向速度 (m/s)
 * @param v_body_y   车体系侧向速度 (m/s), 差速底盘通常为0
 * @param v_body_wz  车体系旋转角速度 (rad/s), 可直接使用 IMU 的 Z 轴角速度
 */
void ESKF_App_Odom_Handler(eskf_t *eskf, float v_body_x, float v_body_y, float v_body_wz);

/**
 * @brief Lidar/UWB/SLAM 相对定位数据处理回调函数 (直接输入 x, y 坐标)
 *        可用于低频更新 (1Hz ~ 20Hz)
 * 
 * @param eskf     ESKF 实例指针
 * @param pos_x    相对原点的 X 坐标 (m) - ENU 坐标系
 * @param pos_y    相对原点的 Y 坐标 (m) - ENU 坐标系
 * @param yaw_rad  航向角 (rad), ENU 坐标系 (0=East, pi/2=North)
 * @param has_yaw  是否包含有效航向 (1: 有, 0: 无)
 * @param timestamp 测量发生的系统时间戳 (s). 用于处理延迟/回溯.
 */
void ESKF_App_Lidar_Handler(eskf_t *eskf, float pos_x, float pos_y, float yaw_rad, uint8_t has_yaw, float timestamp);

/**
 * @brief 获取当前的融合状态
 * @param eskf      ESKF 实例指针
 * @param out_state 输出状态结构体指针
 */
void ESKF_App_Get_State(eskf_t *eskf, eskf_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif // ESKF_H

/* ESKF 算法库总头文件 */
/* 
 * 使用说明:
 * 1. 包含此头文件
 * 2. 使用 eskf_default_config() 获取默认配置
 * 3. 根据需要修改 config 参数
 * 4. 调用 eskf_init() 初始化
 * 5. 在高频循环/中断中调用 eskf_predict() (输入 IMU 数据)
 * 6. 在低频任务中调用 eskf_update_pos() (输入 GNSS/Lidar 数据)
 * 7. 使用 eskf_get_state() 获取当前状态
 */


// 矩阵含义速查：
// P: 状态协方差矩阵 (State Covariance) - 描述当前估计的不确定度
// Q: 过程噪声 (Process Noise) - 描述模型的不确定度 (IMU噪声)
// R: 观测噪声 (Measurement Noise) - 描述观测数据的不确定度 (GNSS误差)


// ============================================================================
//                              ESKF Library Usage Guide
// ============================================================================
// 
// 1. 系统要求 / System Requirements
//    - 处理器: STM32F407 或更高性能 MCU (需 FPU 支持)
//    - 内存: 约 4KB RAM (缓冲区取决于 history size)
// 
// 2. 坐标系定义 / Coordinate Systems
//    - Body Frame (b):  前-左-上 (X-Y-Z) 或 右-前-上 (根据 IMU 安装方向)
//    - World Frame (w): 东-北-天 (ENU)
//    - 单位: 米 (m), 秒 (s), 弧度 (rad), 米/秒^2 (m/s^2), 弧度/秒 (rad/s)
// 
// 3. FreeRTOS 集成指南 / FreeRTOS Integration
//    ESKF 核心结构体 `eskf_t` 不是线程安全的。如果在多任务环境中使用
//    （例如：高频 IMU 中断/任务调用 predict，低频 GPS 任务调用 update），
//    必须使用互斥锁 (Mutex) 或 临界区 (Critical Section) 进行保护。
// 
//    示例代码 (伪代码):
//    -------------------------------------------------------------------------
//    // 全局变量
//    SemaphoreHandle_t eskf_mutex;
//    eskf_t eskf_instance;
// 
//    // 任务1: IMU 处理 (高优先级, e.g. 100Hz - 400Hz)
//    void IMU_Task(void *params) {
//        eskf_imu_meas_t imu_data;
//        while(1) {
//            // 获取 IMU 数据 (acc: m/s^2, gyro: rad/s)
//            xQueueReceive(imu_queue, &imu_data, portMAX_DELAY);
//            
//            xSemaphoreTake(eskf_mutex, portMAX_DELAY);
//            eskf_predict(&eskf_instance, &imu_data, 0.01f); // dt = 1/freq
//            xSemaphoreGive(eskf_mutex);
//        }
//    }
// 
//    // 任务2: GNSS/观测处理 (低优先级, e.g. 1Hz - 10Hz)
//    void GNSS_Task(void *params) {
//        eskf_gnss_meas_t obs_data;
//        while(1) {
//            // 解析 GPS 数据
//            if (Get_GNSS_Data(&obs_data)) {
//                xSemaphoreTake(eskf_mutex, portMAX_DELAY);
//                eskf_update_pos(&eskf_instance, &obs_data);
//                xSemaphoreGive(eskf_mutex);
//            }
//        }
//    }
// 
//    // 任务3: 状态获取与控制 (定时任务)
//    void Control_Task(void *params) {
//        eskf_state_t current_state;
//        while(1) {
//             xSemaphoreTake(eskf_mutex, portMAX_DELAY);
//             eskf_get_state(&eskf_instance, &current_state);
//             xSemaphoreGive(eskf_mutex);
//             
//             // 使用 current_state.pos, current_state.rot ...
//        }
//    }
//    -------------------------------------------------------------------------
