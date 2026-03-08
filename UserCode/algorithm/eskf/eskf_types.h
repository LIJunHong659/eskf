#ifndef ESKF_TYPES_H
#define ESKF_TYPES_H

#include <stdint.h>
#include <math.h>

// 定义浮点类型，STM32F4通常使用 float (单精度)
typedef float eskf_float_t;

// 向量和四元数定义
typedef struct {
    eskf_float_t x;
    eskf_float_t y;
    eskf_float_t z;
} eskf_vec3_t;

typedef struct {
    eskf_float_t w;
    eskf_float_t x;
    eskf_float_t y;
    eskf_float_t z;
} eskf_quat_t;

// -------------------------------------------------------------
// 传感器数据类型定义 (Sensor Data Types)
// -------------------------------------------------------------

/**
 * @brief IMU 测量数据结构
 *        请确保单位正确，否则滤波效果会严重偏差
 */
typedef struct {
    eskf_float_t timestamp;  // 时间戳 (秒). 推荐使用 64位微秒转成的 float，或者系统启动后的秒数
    
    eskf_vec3_t acc;         // 加速度 (m/s^2). 
                             // 注意: 静止时应输出 ~9.8 (或 (0,0,9.8))
                             // 如果是原始值 (LSB) 或 g值，请先转换为 m/s^2
    
    eskf_vec3_t gyro;        // 角速度 (rad/s). 
                             // 注意: 如果是 degree/s，请乘以 (PI/180)
} eskf_imu_meas_t;

/**
 * @brief 里程计/编码器 测量数据
 *        通常提供 Body Frame 下的速度
 */
typedef struct {
    eskf_float_t timestamp;  // 时间戳 (秒)
    
    eskf_float_t v_body_x;   // 前向速度 (m/s)
    eskf_float_t v_body_y;   // 侧向速度 (m/s). 差速底盘通常为0
    eskf_float_t v_body_wz;  // 旋转角速度 (rad/s). 
} eskf_odom_meas_t;

// -------------------------------------------------------------
// 滤波器状态定义 (Filter State)
// -------------------------------------------------------------

#define ESKF_STATE_DIM 15

/**
 * @brief 滤波器名义状态 (Nominal State)
 */
typedef struct {
    eskf_vec3_t pos;        // 位置 (m) - World Frame (ENU)
    eskf_vec3_t vel;        // 速度 (m/s) - World Frame (ENU)
    eskf_quat_t rot;        // 姿态 (四元数) - Body to World. 通常 w 为实部
    
    eskf_vec3_t bg;         // 陀螺仪零偏 (rad/s)
    eskf_vec3_t ba;         // 加速度计零偏 (m/s^2)
    
} eskf_state_t;

/**
 * @brief 误差状态协方差矩阵 (Error State Covariance)
 *        维度: 15x15
 *        顺序: pos(3), vel(3), theta(3), ba(3), bg(3)
 */
typedef struct {
    eskf_float_t P[ESKF_STATE_DIM * ESKF_STATE_DIM]; // 行优先存储
} eskf_cov_t;

#endif // ESKF_TYPES_H
