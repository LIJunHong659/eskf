#ifndef ESKF_CORE_H
#define ESKF_CORE_H

#include "eskf_types.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif


// ESKF 配置参数
typedef struct {
    // -------------------------------------------------------------------------
    // 1. 过程噪声 (Process Noise) - 调参核心
    // -------------------------------------------------------------------------
    // 增大噪声值 = 降低对该传感器的信任 = 更加依赖其他观测 = 响应更平滑但滞后
    // 减小噪声值 = 增加对该传感器的信任 = 响应更快但更敏感
    
    eskf_float_t acc_noise;         // 加速度计噪声标度 (m/s^2)^2. 推荐: 0.1 ~ 0.2
    eskf_float_t gyro_noise;        // 陀螺仪噪声标度 (rad/s)^2.   推荐: 0.01 ~ 0.05
    
    eskf_float_t acc_bias_noise;    // Acc 零偏游走. 推荐: 1e-4 ~ 1e-3
    eskf_float_t gyro_bias_noise;   // Gyro 零偏游走. 推荐: 1e-5 ~ 1e-4
    
    // -------------------------------------------------------------------------
    // 2. 初始不确定性 (Initial Uncertainty) - 影响收敛速度
    // -------------------------------------------------------------------------
    eskf_float_t init_pos_unc;      // 初始位置不确定性 (m)
    eskf_float_t init_vel_unc;      // 初始速度不确定性 (m/s)
    eskf_float_t init_att_unc;      // 初始姿态不确定性 (rad)
    eskf_float_t init_bias_unc;     // 初始零偏不确定性
    
    // -------------------------------------------------------------------------
    // 3. 观测噪声 (Measurement Noise)
    // -------------------------------------------------------------------------
    eskf_float_t pos_noise;         // 位置观测噪声 (m)^2. 若是 Lidar/GNSS 均可使用
    eskf_float_t pos_yaw_noise;     // 航向观测噪声 (rad)^2. 
    
    eskf_float_t odom_vel_x_noise;  // 里程计X轴速度噪声 (m/s)^2
    eskf_float_t odom_vel_y_noise;  // 里程计Y轴速度噪声 (m/s)^2
    eskf_float_t odom_wz_noise;     // 里程计角速度噪声 (rad/s)^2

    // -------------------------------------------------------------------------
    // 4. 环境参数
    // -------------------------------------------------------------------------
    eskf_vec3_t gravity;            //重力向量，通常 {0, 0, 9.81}
} eskf_config_t;

// 通用观测数据结构 (位置 + 航向) - 适用于 GNSS、Lidar SLAM 或 UWB
typedef struct {
    eskf_float_t x;             // Local ENU x (m)
    eskf_float_t y;             // Local ENU y (m)
    eskf_float_t yaw;           // 航向 (rad), ENU 坐标系下通常 0=East, pi/2=North.
    
    eskf_float_t timestamp;     // 数据采集时刻的时间戳 (必须与 IMU 时间戳同一基准)
    
    uint8_t has_yaw;            // 是否包含有效的航向角信息 (1: 有, 0: 无)
} eskf_pos_meas_t;

// ESKF 主类
typedef struct {
    eskf_state_t X;                     // 名义状态
    eskf_cov_t P;                       // 误差协方差 P
    eskf_config_t config;               // 配置副本
    
    uint8_t is_initialized;             // 是否初始化完成
    eskf_float_t last_success_update_time; // 上次成功更新的时间

    // 数据缓冲区 (用于处理观测延迟)
#define ESKF_BUF_SIZE 50
    eskf_imu_meas_t imu_buf[ESKF_BUF_SIZE];
    eskf_state_t X_buf[ESKF_BUF_SIZE]; // 状态历史
    eskf_cov_t   P_buf[ESKF_BUF_SIZE]; // 协方差历史
    eskf_float_t t_buf[ESKF_BUF_SIZE]; // 时间戳
    
    int buf_head;    
    int buf_count;   
    int buf_tail; 
    
    // 辅助变量
    eskf_vec3_t  last_gyro;             // 上一次 IMU Gyro (用于 Odom 预积分插值)
    eskf_vec3_t  last_acc;              // 上一次 IMU Acc
} eskf_t;

// --------------------------------------------------------------------
// API 接口
// --------------------------------------------------------------------

/**
 * @brief 初始化 ESKF 滤波器
 * @param eskf 滤波器句柄
 * @param config 配置参数 (可先用 eskf_default_config 填充)
 * @return 0: 成功, -1: 失败
 */
int eskf_init(eskf_t *eskf, const eskf_config_t *config);

/**
 * @brief 预测 (Propagate) - 应当在 IMU 数据到达时调用
 *        频率建议: 100Hz ~ 400Hz
 * @param eskf 句柄
 * @param imu IMU 数据 (acc: m/s^2, gyro: rad/s)
 * @param dt 距离上一次预测的时间间隔 (s)
 */
void eskf_predict(eskf_t *eskf, const eskf_imu_meas_t *imu, eskf_float_t dt);

/**
 * @brief 位置观测更新 (Correct) - 适用于 GNSS/Lidar/UWB/Vslam 等绝对定位数据
 *        支持延迟测量补偿 (需确保 meas.timestamp 准确)
 * @param eskf 句柄
 * @param meas 测量数据
 */
void eskf_update_pos(eskf_t *eskf, const eskf_pos_meas_t *meas);

/**
 * @brief 里程计速度更新 (Correct) - 应当在 编码器/Odom 数据到达时调用
 * @param eskf 句柄
 * @param meas 里程计数据
 */
void eskf_update_odom(eskf_t *eskf, const eskf_odom_meas_t *meas);

/**
 * @brief 获取当前最新状态 (State Access)
 * @param eskf 句柄
 * @param out_state 输出状态结构体
 */
void eskf_get_state(const eskf_t *eskf, eskf_state_t *out_state);

/**
 * @brief 检查滤波器是否已初始化
 */
int eskf_is_initialized(const eskf_t *eskf);

#ifdef __cplusplus
}
#endif

#endif // ESKF_CORE_H
