/**
 * @file    LocESKF.hpp
 * @brief   ESKF-based Localization System Implementation
 * @author  syhanjin
 * @date    2026-03-08
 */
#pragma once

#include "ILoc.hpp"
#include "eskf.h"
#include <string.h>

namespace chassis_loc
{

/**
 * @brief ESKF定位系统实现类
 * 
 * 特性:
 * - 继承ILoc接口，与底盘系统无缝集成
 * - 封装ESKF算法库，提供多传感器融合能力
 * - 支持IMU(高频)、轮速计(中频)、Lidar/SLAM(低频)数据融合
 * - 自动同步ESKF状态到ILoc的posture和velocity
 */
class LocESKF : public ILoc
{
public:
    LocESKF();
    ~LocESKF() override = default;

    // 禁止拷贝和赋值
    LocESKF(const LocESKF&) = delete;
    LocESKF& operator=(const LocESKF&) = delete;

    /**
     * @brief 初始化ESKF滤波器
     * @param config ESKF配置参数，传nullptr使用默认配置
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool Init(const eskf_config_t* config = nullptr);

    /**
     * @brief 检查是否已初始化
     */
    [[nodiscard]] bool IsInitialized() const;

    // -------------------------------------------------------------------------
    // 传感器数据输入接口 (应在对应的中断或任务中调用)
    // -------------------------------------------------------------------------

    /**
     * @brief IMU数据处理 (建议100Hz~400Hz)
     * @param acc_raw 三轴加速度原始数据 (m/s^2)
     * @param gyro_raw 三轴角速度原始数据 (rad/s)
     * @param dt 距离上一次调用的时间间隔 (s)
     * @param timestamp 数据采集时刻的系统时间戳 (s)
     */
    void UpdateIMU(float acc_raw[3], float gyro_raw[3], float dt, double timestamp);

    /**
     * @brief 轮速计/编码器数据处理 (建议20Hz~100Hz)
     * @param v_body_x 车体系前向速度 (m/s)
     * @param v_body_y 车体系侧向速度 (m/s)，差速底盘通常为0
     * @param v_body_wz 车体系旋转角速度 (rad/s)
     */
    void UpdateOdom(float v_body_x, float v_body_y, float v_body_wz, float timestamp);

    /**
     * @brief Lidar/UWB/SLAM绝对定位数据处理 (建议1Hz~20Hz)
     * @param pos_x 相对原点的X坐标 (m) - ENU坐标系
     * @param pos_y 相对原点的Y坐标 (m) - ENU坐标系
     * @param yaw_rad 航向角 (rad)，ENU坐标系 (0=East, pi/2=North)
     * @param has_yaw 是否包含有效航向 (1:有, 0:无)
     * @param timestamp 测量发生的系统时间戳 (s)
     */
    void UpdateLidar(float pos_x, float pos_y, float yaw_rad, uint8_t has_yaw, float timestamp);

    // -------------------------------------------------------------------------
    // 状态获取接口
    // -------------------------------------------------------------------------

    /**
     * @brief 获取当前ESKF完整状态
     * @param out_state 输出状态结构体
     */
    void GetState(eskf_state_t* out_state) const;

    /**
     * @brief 获取ESKF实例指针 (用于高级操作)
     */
    [[nodiscard]] eskf_t* GetESKFInstance();

    /**
     * @brief 从ESKF状态同步更新ILoc的posture和velocity
     *        应在每次ESKF更新后调用，或周期性调用
     */
    void SyncStateFromESKF();

private:
    eskf_t eskf_instance_;
    bool initialized_{false};
    
    // 保存最新的角速度 (rad/s)，用于状态同步（可由四元数换算而来）
    float latest_wz_rad_{0.0f};
};

} // namespace chassis_loc
