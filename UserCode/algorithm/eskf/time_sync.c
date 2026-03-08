#include "time_sync.h"
#include <math.h>

void time_sync_init(time_sync_t *sync, float alpha_fast, float alpha_slow) {
    if (!sync) return;
    sync->offset = 0.0;
    sync->alpha_fast = alpha_fast;
    sync->alpha_slow = alpha_slow;
    sync->is_initialized = 0;
    sync->last_update_time = 0.0;
}

void time_sync_update(time_sync_t *sync, double t_pc_send, double t_stm_rx) {
    if (!sync) return;

    // 计算当前的观测偏移量
    // measured_offset = T_stm_now - T_pc_now
    // 其中 t_stm_rx 包含 传输延迟，所以 measured_offset 略大于 真实时钟偏移
    double measured_offset = t_stm_rx - t_pc_send;

    if (!sync->is_initialized) {
        sync->offset = measured_offset;
        sync->is_initialized = 1;
        sync->last_update_time = t_stm_rx;
        return;
    }

    // 计算误差
    double error = measured_offset - sync->offset;

    // 非对称滤波
    if (error < 0.0) {
        // Case 1: measured_offset < current_offset
        // 意味着本次传输延迟比预估的要小（更接近物理极限），或者 STM32 跑慢了/PC 跑快了
        // 这是一个“高质量”的测量值，应该快速更新以逼近真实下界
        sync->offset += sync->alpha_fast * error;
    } else {
        // Case 2: measured_offset > current_offset
        // 意味着本次传输延迟很大（Jitter），或者时钟漂移方向相反
        // 这通常由操作系统调度、USB缓冲等引起，属于噪声
        // 应该慢速更新，仅仅跟踪时钟漂移，不要被噪声带偏
        sync->offset += sync->alpha_slow * error;
    }

    sync->last_update_time = t_stm_rx;
}

double time_sync_get_aligned_time(time_sync_t *sync, double t_pc_capture) {
    if (!sync || !sync->is_initialized) {
        // 如果未初始化，直接返回原值（或 0 报错，视需求而定）
        return t_pc_capture; 
    }
    
    // T_stm_aligned = T_pc + Offset
    return t_pc_capture + sync->offset;
}

/* 
 * 附加说明：关于 4 个关键时间点
 * 
 * T0: FastLIO2 开始采集 (Lidar Scan Start) -> t_pc_capture
 * T1: FastLIO2 处理完成 (Algorithm Done)
 * T2: PC 串口发送时刻 (Send to DMA) -> t_pc_send
 * T3: STM32 串口整帧接收时刻 (IRQ Handler) -> t_stm_rx
 * 
 * 关系：
 * T_stm_rx = T_pc_send + Clock_Diff + Trans_Delay
 * 
 * 我们需要求的是 Clock_Diff。
 * 通过 Asymmetric Low Pass Filter, 我们能估算出 (Clock_Diff + Min_Trans_Delay)。
 * 
 * 最终应用：
 * T_stm_capture_valid = t_pc_capture + (Clock_Diff + Min_Trans_Delay)
 *                     = t_pc_capture + sync->offset
 * 
 * 这样得到的 T_stm_capture_valid 实际上比 "绝对物理时间" 晚了 Min_Trans_Delay。
 * 但在单向通信系统中，这是我们能做到的最佳对齐（因果性要求）。
 * 对于 ESKF 回溯来说，只要 IMU 和 Lidar 在同一个时间轴上相对位置正确即可。
 */
