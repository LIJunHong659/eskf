#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 时间同步模块
 * 用于估算上位机(HOST)与下位机(SLAVE/STM32)之间的时间偏移
 * 算法基于非对称低通滤波 (Asymetric Low-Pass Filter)
 * 目标：Offset = T_stm32 - T_pc
 * T_stm32_aligned = T_pc + Offset
 */

typedef struct {
    double  offset;             // 当前估算的时间偏移 (STM32时间 - PC时间)（使用double防止精度丢失）
    float   alpha_fast;         // 快速跟随系数 (当测量延迟变小时) -> 0.1 ~ 0.5
    float   alpha_slow;         // 慢速跟随系数 (当测量延迟变大时，即Jitter) -> 0.0001 ~ 0.005
    uint8_t is_initialized;     // 是否已初始化
    double  last_update_time;   // 上次更新时间 (STM32时间)
} time_sync_t;

/**
 * @brief 初始化时间同步器
 * @param sync 句柄
 * @param alpha_fast 建议 0.1 (快速收敛)
 * @param alpha_slow 建议 0.001 (抑制抖动)
 */
void time_sync_init(time_sync_t *sync, float alpha_fast, float alpha_slow);

/**
 * @brief 更新时间偏移估计
 * 
 * 核心逻辑：非对称滤波
 * 我们假设传输延迟是正值且存在一个物理最小值。
 * 我们测量的 raw_offset = t_stm_rx - t_pc_send = (T_stm_clock - T_pc_clock) + Transmission_Delay
 * 
 * 真正的 Clock_Offset = (T_stm_clock - T_pc_clock)
 * 我们实际上估算的是：Estimate = Clock_Offset + Min_Transmission_Delay
 * 
 * 如果 raw_offset < Estimate，说明这次传输特别快，或者是时钟漂移导致 Gap 变大了
 * -> 这更接近真实情况，需要 Fast Update
 * 
 * 如果 raw_offset > Estimate，说明这次传输发生了拥塞/调度延迟
 * -> 这是噪声，需要 Slow Update
 * 
 * @param sync 句柄
 * @param t_pc_send 上位机发送时刻的时间戳 (注意：不是采集时刻，是串口发送时刻)
 * @param t_stm_rx  下位机接收中断时刻的时间戳 (STM32本地时间)
 */
void time_sync_update(time_sync_t *sync, double t_pc_send, double t_stm_rx);

/**
 * @brief 将上位机采集时间转换为下位机对齐时间
 * 
 * @param sync 句柄
 * @param t_pc_capture 上位机原始采集时间 (FastLIO2 PointCloud Timestamp)
 * @return double 对齐后的下位机时间
 */
double time_sync_get_aligned_time(time_sync_t *sync, double t_pc_capture);

#ifdef __cplusplus
}
#endif

#endif // TIME_SYNC_H
