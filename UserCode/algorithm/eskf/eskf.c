#include "eskf.h"
#include <math.h>
#include <string.h>

// ----------------------------------------------------------------------------
// 全局变量定义
// ----------------------------------------------------------------------------




// ----------------------------------------------------------------------------
// API 实现
// ----------------------------------------------------------------------------

void ESKF_App_Init(eskf_t *eskf) {
    eskf_config_t config;
    
    // 0. 清零配置结构体 (防止栈内存残留垃圾数据)
    memset(&config, 0, sizeof(eskf_config_t));

    // =======================================================
    // 1. 手动配置所有参数
    // =======================================================
    
    // --- A. 过程噪声 (Process Noise) ---
    // 描述 IMU 传感器的“可信度”。值越小，相信 IMU 积分越多。
    // 如果发现滤波结果滞后，减小这些值；如果结果震荡/发散，增大这些值。
    config.acc_noise       = 0.02f;        
    config.gyro_noise      = 0.002f;       
    config.acc_bias_noise  = 1e-4f;        
    config.gyro_bias_noise = 1e-5f;
    
    // --- B. 观测噪声 (Measurement Noise) ---
    // 描述外部观测数据的“可信度”。
    
    // Lidar SLAM 定位噪声 (单位: m^2)
    // 根据 FastLIO2 精度调节，通常 1mm ~ 5cm 级别
    // 0.05m * 0.05m = 0.0025. 设为 0.01 比较保守
    config.pos_noise  = 0.0025f;      
    
    // Lidar SLAM 航向噪声 (单位: rad^2)
    // FastLIO2 角度通常很准
    config.pos_yaw_noise  = 0.001f;        

    // 里程计噪声 (如果用到)
    // 轮速计容易打滑，可以给大一点
    config.odom_vel_x_noise  = 0.05f;         // (m/s)^2
    config.odom_vel_y_noise  = 0.05f;         // (m/s)^2
    config.odom_wz_noise   = 0.05f;         // (rad/s)^2
    
    // --- C. 初始状态不确定性 (Initial Uncertainty) ---
    // FastLIO2 启动时刻即为原点，所以初始不确定性可以给小一点
    config.init_pos_unc    = 0.1f;         
    config.init_vel_unc    = 0.1f;         
    config.init_att_unc    = 0.1f;         
    config.init_bias_unc   = 0.1f;         
    
    // --- D. 环境参数 ---
    config.gravity.x       = 0.0f;
    config.gravity.y       = 0.0f;
    config.gravity.z       = 9.81f;        // 当地重力加速度

    // 2. 初始化滤波器
    eskf_init(eskf, &config);
}

void ESKF_App_IMU_Handler(eskf_t *eskf, float acc_raw[3], float gyro_raw[3], float dt, double timestamp) {
    // 准备数据结构
    eskf_imu_meas_t imu_data;

    // 1. 获取时间戳 (必须与 GNSS 时间戳同一基准)
    imu_data.timestamp = timestamp; // 使用传入的采集时间

    // 2. 单位转换 (这是最容易出错的地方!)
    // ESKF 要求 Acc 单位: m/s^2, Gyro 单位: rad/s
    // 假设输入 acc_raw 是 g (1g = 9.8m/s^2), gyro_raw 是 deg/s
    
    imu_data.acc.x = acc_raw[0] * CONST_G;
    imu_data.acc.y = acc_raw[1] * CONST_G;
    imu_data.acc.z = acc_raw[2] * CONST_G;

    imu_data.gyro.x = gyro_raw[0] * DEG2RAD_F;
    imu_data.gyro.y = gyro_raw[1] * DEG2RAD_F;
    imu_data.gyro.z = gyro_raw[2] * DEG2RAD_F;

    // 3. 执行预测
    // 注意: 多线程环境下这里需要加锁
    eskf_predict(eskf, &imu_data, dt);
}


void ESKF_App_Odom_Handler(eskf_t *eskf, float v_body_x, float v_body_y, float v_body_wz, float timestamp) {
    eskf_odom_meas_t odom;
    // 使用传入的时间戳，确保与 IMU 数据使用同一时间基准
    odom.timestamp = timestamp;
    
    odom.v_body_x  = v_body_x;
    odom.v_body_y  = v_body_y;
    odom.v_body_wz = v_body_wz;

    eskf_update_odom(eskf, &odom);
}

void ESKF_App_Lidar_Handler(eskf_t *eskf, float pos_x, float pos_y, float yaw_rad, uint8_t has_yaw, float timestamp) {
    eskf_pos_meas_t meas;
    
    // 使用传入的准确时间戳 (需已对齐到系统时间)
    meas.timestamp = timestamp;
    
    // 直接传入 ENU 局部坐标
    meas.x = pos_x;
    meas.y = pos_y;
    meas.yaw = yaw_rad;
    meas.has_yaw = has_yaw;
    
    // 如果 Lidar SLAM 已经给出了相对原点的坐标，直接更新即可
    // 注意: 请确认 pos_x, pos_y 的原点与 ESKF 的原点是否一致
    // 如果是第一次启动，这个 meas 的位置可能就是新的世界坐标系原点（取决于 eskf_core 内部初始化逻辑）
    eskf_update_pos(eskf, &meas);
}


void ESKF_App_Get_State(eskf_t *eskf, eskf_state_t *out_state) {

    if (out_state != NULL) {
        // 注意: 多线程环境下这里需要加锁
        eskf_get_state(eskf, out_state);
    }
}
