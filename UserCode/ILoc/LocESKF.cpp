/**
 * @file    LocESKF.cpp
 * @brief   ESKF-based Localization System Implementation
 */
#include "LocESKF.hpp"

namespace chassis_loc
{

LocESKF::LocESKF()
{
    // 清零ESKF实例
    std::memset(&eskf_instance_, 0, sizeof(eskf_t));
}

bool LocESKF::Init(const eskf_config_t* config)
{
    if (initialized_) {
        return true; // 已初始化
    }

    if (config != nullptr) {
        // 使用用户提供的配置
        int result = eskf_init(&eskf_instance_, config);
        if (result == 0) {
            initialized_ = true;
            SyncStateFromESKF();
        }
    } else {
        // 使用 ESKF_App_Init 中的默认配置
        ESKF_App_Init(&eskf_instance_);
        initialized_ = eskf_is_initialized(&eskf_instance_) != 0;
        if (initialized_) {
            SyncStateFromESKF();
        }
    }

    return initialized_;
}

bool LocESKF::IsInitialized() const
{
    return initialized_;
}

void LocESKF::UpdateIMU(float acc_raw[3], float gyro_raw[3], float dt, double timestamp)
{
    if (!initialized_) {
        return;
    }

    // 保存最新的Z轴角速度（注意：输入是 deg/s，需要转换为 rad/s 用于内部计算）
    // ESKF_App_IMU_Handler 内部会进行 deg->rad 转换
    latest_wz_rad_ = gyro_raw[2] * DEG2RAD_F;

    // 使用应用层封装函数，自动处理单位转换（g->m/s^2, deg/s->rad/s）
    ESKF_App_IMU_Handler(&eskf_instance_, acc_raw, gyro_raw, dt, timestamp);
    
    // 同步更新后的状态到ILoc
    SyncStateFromESKF();
}

void LocESKF::UpdateOdom(float v_body_x, float v_body_y, float v_body_wz)
{
    if (!initialized_) {
        return;
    }

    // 使用应用层封装函数
    ESKF_App_Odom_Handler(&eskf_instance_, v_body_x, v_body_y, v_body_wz);
    
    // 同步更新后的状态到ILoc
    SyncStateFromESKF();
}

void LocESKF::UpdateLidar(float pos_x, float pos_y, float yaw_rad, uint8_t has_yaw, float timestamp)
{
    if (!initialized_) {
        return;
    }

    // 使用应用层封装函数
    ESKF_App_Lidar_Handler(&eskf_instance_, pos_x, pos_y, yaw_rad, has_yaw, timestamp);
    
    // 同步更新后的状态到ILoc
    SyncStateFromESKF();
}

void LocESKF::GetState(eskf_state_t* out_state) const
{
    if (out_state != nullptr && initialized_) {
        // 注意：ESKF_App_Get_State 需要非 const 指针，但 GetState 是 const 方法
        // 使用 const_cast 是安全的，因为 ESKF_App_Get_State 不会修改 eskf 状态
        ESKF_App_Get_State(const_cast<eskf_t*>(&eskf_instance_), out_state);
    }
}

eskf_t* LocESKF::GetESKFInstance()
{
    return &eskf_instance_;
}

void LocESKF::SyncStateFromESKF()
{
    if (!initialized_) {
        return;
    }

    eskf_state_t state;
    ESKF_App_Get_State(&eskf_instance_, &state);

    // 同步世界坐标系位置 (ENU -> 车体坐标系)
    // ESKF使用ENU (东-北-上)，你的ILoc使用 (前-左-上)
    // 这里假设两者一致，如有需要请添加坐标转换
    posture_.in_world.x = state.pos.x;
    posture_.in_world.y = state.pos.y;
    
    // 四元数转yaw角 (ENU坐标系)
    // q = w + xi + yj + zk
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    float w = state.rot.w;
    float x = state.rot.x;
    float y = state.rot.y;
    float z = state.rot.z;
    float yaw_rad = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    posture_.in_world.yaw = yaw_rad * RAD2DEG_F; // 转换为度

    // 同步世界坐标系速度
    velocity_.in_world.vx = state.vel.x;
    velocity_.in_world.vy = state.vel.y;
    // 角速度：使用最新的IMU Z轴角速度（已转换为 rad/s）
    velocity_.in_world.wz = latest_wz_rad_ * RAD2DEG_F; // rad/s -> deg/s

    // 计算车体坐标系速度 (通过坐标变换)
    velocity_.in_body = WorldVelocity2BodyVelocity(velocity_.in_world);
}

} // namespace chassis_loc
