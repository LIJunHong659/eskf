#ifndef APP_HPP
#define APP_HPP

#include "cmsis_os2.h"
#include "eskf.h"
#include "cmsis_os.h"
#include "LocESKF.hpp"
#include "time_sync.h"
#include <cstddef>
#include <cstdio>

// 模拟
// 假设imu数据格式  t, acc[3], gyro[3]
typedef struct{
    float t;
    float acc[3];
    float gyro[3];
}Imu_data_t;
// 假设odom数据格式 update_flag, v_body_x, v_body_y, v_body_wz
typedef struct{
    uint8_t update_flag; // 0: 无效数据, 1: 有效数据
    float v_body_x;   // 车体系前向速度 (m/s)
    float v_body_y;   // 车体系侧向速度 (m/s), 差速底盘通常为0
    float v_body_wz;  // 车体系旋转角速度 (rad/s), 可直接使用 IMU 的 Z 轴角速度
}Odom_data_t;
// 假设lidar数据格式 start_t, pos_x, pos_y, yaw
typedef struct{
    float start_t;
    float pos_x;
    float pos_y;
    float yaw;
}Lidar_data_t;
// 假设存在
void IMU_get_data(Imu_data_t *imu_data);
void Lidar_get_data(Lidar_data_t *lidar_data);
void Odom_get_data(Odom_data_t *odom_data);
float get_current_time(); // 获取当前时间戳，单位秒(高精度)

// 中断处理函数声明
void IMU_get_interrupt_handler(void *argument);
void Odom_get_interrupt_handler(void *argument);
void Lidar_get_interrupt_handler(void *argument);
#endif // APP_HPP