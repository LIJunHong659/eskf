#include "eskf_core.h"
#include "eskf_math.h"
#include "eskf_mat.h"
#include <string.h>
#include <math.h>

#define CONST_PI 3.1415926535f

// ==========================================
// 辅助函数声明
// ==========================================
static void propagate_one_step(eskf_t *eskf, const eskf_imu_meas_t *meas, float dt);
static float normalize_angle(float angle);
static float quat_to_yaw(eskf_quat_t q);

// ==========================================
// 接口实现
// ==========================================

int eskf_init(eskf_t *eskf, const eskf_config_t *config) {
    if (!eskf || !config) return -1;
    
    // 验证配置参数
    if (config->acc_noise < 0 || config->gyro_noise < 0 ||
        config->init_pos_unc < 0 || config->init_vel_unc < 0 ||
        config->init_att_unc < 0 || config->init_bias_unc < 0) {
        return -2; // 无效的配置参数
    }
    
    memset(eskf, 0, sizeof(eskf_t));
    eskf->config = *config;
    
    // 初始化状态
    eskf->X.rot.w = 1.0f; // Unit Quaternion
    // 其他状态变量保持为0，或根据需要设置合理的默认值
    
    // 初始化协方差 P0 (对角阵)
    mat_zero(eskf->P.P, ESKF_STATE_DIM, ESKF_STATE_DIM);
    for (int i = 0; i < ESKF_STATE_DIM; i++) {
        float var;
        if (i < 2) var = config->init_pos_unc;      // 只初始化x,y位置
        else if (i < 4) var = config->init_vel_unc; // 只初始化x,y速度
        else if (i < 5) var = config->init_att_unc; // 只初始化yaw姿态
        else var = config->init_bias_unc;
        
        eskf->P.P[i * ESKF_STATE_DIM + i] = var * var; // 使用宏定义
    }

    // 初始化里程计数据有效性标记和时间戳缓冲区
    for (int i = 0; i < ESKF_BUF_SIZE; i++) {
        eskf->odom_valid[i] = 0;
        eskf->odom_timestamp_buf[i] = 0.0f;
    }

    // 初始化最新里程计数据标记
    eskf->has_latest_odom = 0;
    eskf->last_success_update_time = 0.0f; // 初始化时间戳

    eskf->is_initialized = 1;
    return 0; // Success
}

// -------------------------------------------------------------------------
// 预测流程 (高频调用)
// -------------------------------------------------------------------------

void eskf_predict(eskf_t *eskf, const eskf_imu_meas_t *imu, eskf_float_t dt) {
    if (!eskf->is_initialized) return;

    // 1. 状态积分与协方差预测
    propagate_one_step(eskf, imu, dt);

    // 更新辅助变量
    eskf->last_gyro = imu->gyro;
    eskf->last_acc = imu->acc;

    // 2. 存入循环队列 (用于延时观测补偿)
    int idx = eskf->buf_head;
    eskf->X_buf[idx]   = eskf->X;
    eskf->P_buf[idx]   = eskf->P;
    eskf->imu_buf[idx] = *imu;     
    eskf->t_buf[idx]   = imu->timestamp;
    
    // 注意：里程计数据现在在 eskf_update_odom 中存储
    // 这里只存储 IMU 数据和状态
    
    eskf->buf_head = (eskf->buf_head + 1) % ESKF_BUF_SIZE;
    if (eskf->buf_count < ESKF_BUF_SIZE) {
        eskf->buf_count++;
    } else {
        eskf->buf_tail = (eskf->buf_tail + 1) % ESKF_BUF_SIZE; // 覆盖最老
    }
}

// -------------------------------------------------------------------------
// 更新流程 (低频调用)
// -------------------------------------------------------------------------

// 查找最接近 t_target 的历史状态索引
static int buffer_find_state_idx(eskf_t *eskf, double t_target) {
    if (eskf->buf_count == 0) return -1;

    // 二分查找优化 (O(log n) 替代 O(n))
    int head = eskf->buf_tail;
    int tail = (eskf->buf_head - 1 + ESKF_BUF_SIZE) % ESKF_BUF_SIZE;
    int count = eskf->buf_count;
    
    // 处理环形缓冲区的边界情况
    if (eskf->t_buf[head] <= eskf->t_buf[tail]) {
        // 缓冲区未环绕，时间戳单调递增
        int left = 0, right = count - 1;
        while (left <= right) {
            int mid = (left + right) / 2;
            int mid_idx = (head + mid) % ESKF_BUF_SIZE;
            double mid_time = eskf->t_buf[mid_idx];
            
            if (mid_time == t_target) {
                return mid_idx; // 精确匹配
            } else if (mid_time < t_target) {
                left = mid + 1;
            } else {
                right = mid - 1;
            }
        }
        
        // 找到最接近的两个点
        int best_idx;
        if (left >= count) {
            best_idx = (head + count - 1) % ESKF_BUF_SIZE;
        } else if (left == 0) {
            best_idx = head;
        } else {
            int idx1 = (head + left - 1) % ESKF_BUF_SIZE;
            int idx2 = (head + left) % ESKF_BUF_SIZE;
            double diff1 = fabs(eskf->t_buf[idx1] - t_target);
            double diff2 = fabs(eskf->t_buf[idx2] - t_target);
            best_idx = (diff1 < diff2) ? idx1 : idx2;
        }
        
        // 检查时间差是否在允许范围内
        if (fabs(eskf->t_buf[best_idx] - t_target) > 0.1) {
            return -1; // Too old or too new
        }
        return best_idx;
    } else {
        // 缓冲区环绕，时间戳分为两部分
        // 先检查目标时间戳在哪个部分
        if (t_target >= eskf->t_buf[head]) {
            // 在头部部分查找
            int left = 0, right = (tail - head + ESKF_BUF_SIZE) % ESKF_BUF_SIZE;
            while (left <= right) {
                int mid = (left + right) / 2;
                int mid_idx = (head + mid) % ESKF_BUF_SIZE;
                double mid_time = eskf->t_buf[mid_idx];
                
                if (mid_time == t_target) {
                    return mid_idx;
                } else if (mid_time < t_target) {
                    left = mid + 1;
                } else {
                    right = mid - 1;
                }
            }
            int best_idx = (head + left - 1) % ESKF_BUF_SIZE;
            if (fabs(eskf->t_buf[best_idx] - t_target) > 0.1) {
                return -1;
            }
            return best_idx;
        } else {
            // 在尾部部分查找
            int left = 0, right = (ESKF_BUF_SIZE - tail - 1);
            while (left <= right) {
                int mid = (left + right) / 2;
                int mid_idx = (tail - mid + ESKF_BUF_SIZE) % ESKF_BUF_SIZE;
                double mid_time = eskf->t_buf[mid_idx];
                
                if (mid_time == t_target) {
                    return mid_idx;
                } else if (mid_time < t_target) {
                    right = mid - 1;
                } else {
                    left = mid + 1;
                }
            }
            int best_idx = (tail - left + 1 + ESKF_BUF_SIZE) % ESKF_BUF_SIZE;
            if (fabs(eskf->t_buf[best_idx] - t_target) > 0.1) {
                return -1;
            }
            return best_idx;
        }
    }
}

void eskf_update_pos(eskf_t *eskf, const eskf_pos_meas_t *meas) {
    if (!eskf->is_initialized) {
        eskf->X.pos.x = meas->x;
        eskf->X.pos.y = meas->y;
        // z位置保持固定
        if (meas->has_yaw) {
            float cy = cosf(meas->yaw * 0.5f);
            float sy = sinf(meas->yaw * 0.5f);
            eskf->X.rot.w = cy;
            eskf->X.rot.x = 0.0f;
            eskf->X.rot.y = 0.0f;
            eskf->X.rot.z = sy;
        }
        eskf->is_initialized = 1;
        return;
    }

    // 1. 延迟补偿: 找到对应时刻的状态
    int state_idx = buffer_find_state_idx(eskf, meas->timestamp);
    if (state_idx < 0) return;
    
    // 2. 取出历史状态作为 Update 的基准
    eskf_state_t X_curr = eskf->X_buf[state_idx];
    eskf_cov_t   *P_ptr = &eskf->P_buf[state_idx]; // Pointer to history Covariance
    float *P = P_ptr->P;

    // 3. 构建观测模型 (Sparse Update)
    int meas_dim = 2 + (meas->has_yaw ? 1 : 0);
    float Y[3]; // Residual
    Y[0] = meas->x - X_curr.pos.x;
    Y[1] = meas->y - X_curr.pos.y;
    if (meas->has_yaw) Y[2] = normalize_angle(meas->yaw - quat_to_yaw(X_curr.rot));

    // H 矩阵稀疏特性: H_pos 选择 Pos(0,1), H_yaw 选择 ThetaZ(4)
    int idx_map[3] = {0, 1, 4};

    // PHt = P * H^T (Extract columns 0, 1, [4] of P)
    float PHt[11 * 3]; 
    for (int c = 0; c < meas_dim; c++) {
        int state_col = idx_map[c];
        for (int r = 0; r < 11; r++) {
            PHt[r * meas_dim + c] = P[r * 11 + state_col];
        }
    }

    // S = H * PHt + R
    float S[9]; 
    for (int r = 0; r < meas_dim; r++) {
        int state_row = idx_map[r]; // Row of H selects Row of PHt
        for (int c = 0; c < meas_dim; c++) {
            S[r * meas_dim + c] = PHt[state_row * meas_dim + c];
        }
        // Add R
        float noise = (r < 2) ? eskf->config.pos_noise : eskf->config.pos_yaw_noise;
        S[r * meas_dim + r] += noise;
    }

    // S^-1
    float Si[9];
    if (mat_inv(S, Si, meas_dim) != 0) return;

    // K = PHt * Si
    float K[11 * 3];
    mat_mul(PHt, Si, K, 11, meas_dim, meas_dim);
    
    // dx = K * Y
    float dx[11];
    mat_mul(K, Y, dx, 11, meas_dim, 1);
    
    // Update P = P - K * (H P)
    // HP (meas_dim x 11) is Transpose of PHt (symmetric P)
    for (int i = 0; i < 11; i++) {
        for (int j = 0; j < 11; j++) {
            float correction = 0.0f;
            for (int k = 0; k < meas_dim; k++) {
                // K[i][k] * HP[k][j] -> HP[k][j] is P[idx_map[k]][j]
                correction += K[i * meas_dim + k] * P[idx_map[k] * 11 + j];
            }
            P[i * 11 + j] -= correction;
        }
    }

    // 5. 误差状态注入
    X_curr.pos.x += dx[0];
    X_curr.pos.y += dx[1];
    // X_curr.pos.z 保持不变
    X_curr.vel.x += dx[2];
    X_curr.vel.y += dx[3];
    // X_curr.vel.z 保持不变
    
    // 只更新yaw，保持roll和pitch固定
    if (fabs(dx[4]) > 1e-6f) {
        float h_angle = dx[4] * 0.5f;
        eskf_quat_t dq = {cosf(h_angle), 0, 0, sinf(h_angle)};
        X_curr.rot = quat_mul(X_curr.rot, dq); 
        quat_normalize(&X_curr.rot);
    }
    X_curr.ba.x += dx[5];  X_curr.ba.y += dx[6]; X_curr.ba.z += dx[7];
    X_curr.bg.x += dx[8]; X_curr.bg.y += dx[9]; X_curr.bg.z += dx[10];

    // 更新 buffer 中的状态
    eskf->X_buf[state_idx] = X_curr;
    
    // 6. 重积分 (Re-propagation)
    // 使用临时上下文进行完整的状态和协方差预测
    // 我们必须从修正后的历史时刻 (state_idx) 开始，一路积分到最新的 buf_head
    eskf_t temp_eskf;
    temp_eskf.config = eskf->config;
    temp_eskf.X = X_curr;       // 使用修正后的历史状态
    temp_eskf.P = *P_ptr;       // 使用修正后的历史协方差
    
    int curr_idx = state_idx;
    int next_idx = (state_idx + 1) % ESKF_BUF_SIZE;
    
    // Safety check loop 
    int loop_guard = 0;
    while (next_idx != eskf->buf_head && loop_guard < ESKF_BUF_SIZE) {
        float dt_segment = eskf->t_buf[next_idx] - eskf->t_buf[curr_idx];
        
        // 简单的 dt 保护，防止时间戳抖动或发散
        if (dt_segment <= 1e-4f) dt_segment = 0.01f; 

        // 获取产生 next_idx 状态的那一帧 IMU 数据
        const eskf_imu_meas_t *meas_seg = &eskf->imu_buf[next_idx];
        
        // 调用标准预测函数，同时更新 temp_eskf.X 和 temp_eskf.P
        propagate_one_step(&temp_eskf, meas_seg, dt_segment);
        
        // 检查该时刻是否有有效的里程计数据
        // 使用最近邻匹配，时间差超过阈值则跳过
        if (eskf->odom_valid[next_idx]) {
            float time_diff = fabs(eskf->odom_timestamp_buf[next_idx] - eskf->t_buf[next_idx]);
            // 时间差阈值：50ms，超过则跳过更新
            if (time_diff < 0.015f) {
                eskf_update_odom_internal(&temp_eskf, &eskf->odom_buf[next_idx], meas_seg);
            }
        }
        
        // 可选：更新历史 Buffer 中的值，这样如果未来还有针对该时间段的观测，
        // 能基于更准确的状态进行更新。
        eskf->X_buf[next_idx] = temp_eskf.X;
        eskf->P_buf[next_idx] = temp_eskf.P;
        
        curr_idx = next_idx;
        next_idx = (next_idx + 1) % ESKF_BUF_SIZE;
        loop_guard++;
    }
    
    // 最后更新当前时刻的滤波状态
    eskf->X = temp_eskf.X;
    eskf->P = temp_eskf.P;
    
    eskf->last_success_update_time = meas->timestamp;
}

// -------------------------------------------------------------------------

// helper: 从四元数构建旋转矩阵 (Row-major)
static void quat_to_rotmat(eskf_quat_t q, float *R) {
    float q0=q.w, q1=q.x, q2=q.y, q3=q.z;
    float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;
    float q0q3=q0*q3, q1q2=q1*q2;

    R[0] = q0q0+q1q1-q2q2-q3q3; R[1] = 2.0f*(q1q2-q0q3);    R[2] = 2.0f*(q1*q3+q0*q2);
    R[3] = 2.0f*(q1q2+q0q3);    R[4] = q0q0-q1q1+q2q2-q3q3; R[5] = 2.0f*(q2*q3-q0*q1);
    R[6] = 2.0f*(q1*q3-q0*q2);  R[7] = 2.0f*(q2*q3+q0*q1);  R[8] = q0q0-q1q1-q2q2+q3q3;
}

void eskf_update_odom(eskf_t *eskf, const eskf_odom_meas_t *meas) {
    if (!eskf->is_initialized) return;

    // 更新最新里程计数据
    eskf->latest_odom = *meas;
    eskf->has_latest_odom = 1;
    
    // 找到与里程计数据时间戳最接近的 IMU 数据位置
    int odom_idx = buffer_find_state_idx(eskf, meas->timestamp);
    if (odom_idx >= 0) {
        // 存储里程计数据到对应的缓冲区位置
        eskf->odom_buf[odom_idx] = *meas;
        eskf->odom_timestamp_buf[odom_idx] = meas->timestamp;
        eskf->odom_valid[odom_idx] = 1;
    }

    // 1. 延迟补偿: 查找历史状态
    int state_idx = buffer_find_state_idx(eskf, meas->timestamp);
    if (state_idx < 0) return;

    eskf_state_t X_curr = eskf->X_buf[state_idx];
    eskf_cov_t   *P_ptr = &eskf->P_buf[state_idx];
    float *P = P_ptr->P;

    // 2. 准备数据
    // R: Rotation from Body to World (3x3)
    float R[9];
    quat_to_rotmat(X_curr.rot, R);
    
    // v_w: World Velocity
    float v_w[3] = {X_curr.vel.x, X_curr.vel.y, X_curr.vel.z};

    // 获取对应时刻的 IMU 数据 (用于计算角速度残差)
    const eskf_imu_meas_t *imu_meas = &eskf->imu_buf[state_idx];

    // 3. 预测观测 (Predict Measurement)
    // v_b_pred = R^T * v_w
    // wz_pred  = gyro_z - bg_z
    float v_b_pred[3];
    float R_inv[9]; // R^T
    mat_trans(R, R_inv, 3, 3);
    
    // v_body = R^T * v_world
    for(int i=0; i<3; i++) {
        v_b_pred[i] = R_inv[i*3+0] * v_w[0] + R_inv[i*3+1] * v_w[1] + R_inv[i*3+2] * v_w[2];
    }

    // 4. 残差 (Residual)
    // 融合: Vx, Vy, Wz
    float Y[3];
    Y[0] = meas->v_body_x - v_b_pred[0];
    Y[1] = meas->v_body_y - v_b_pred[1];
    Y[2] = meas->v_body_wz - (imu_meas->gyro.z - X_curr.bg.z);

    // 5. 雅可比矩阵 H (3 x 12)
    // H 结构:
    // Rows 0,1 (Vel): 对 Vel 和 Att (yaw) 有非零块
    // Row  2   (Wz) : 对 Bg 有非零块 (H_wz_bg = -1, 因为 pred = gyro - bg, so d(pred)/d(bg) = -1)
    
    // 计算 H_theta = R^T * [v_w]x (只yaw分量)
    float vx_skew[9] = {
           0,   -v_w[2],  v_w[1],
         v_w[2],     0,  -v_w[0],
        -v_w[1],  v_w[0],     0
    };
    
    // H_theta = R^T * [v_w]x
    float H_theta[9]; 
    mat_mul(R_inv, vx_skew, H_theta, 3, 3, 3);
    
    // 用 H_vel 表示 R^T (2x3部分)
    // 用 H_att 表示 H_theta (2x1部分，只yaw)

    // PHt = P * H^T  (11 x 3)
    float PHt[33]; // 11 rows, 3 cols
    
    for (int r = 0; r < 11; r++) {
        float *row_P = &P[r*11];
        
        // --- Col 0 (Meas Vx) ---
        // H = [0, R^T[0], R^T[0]*[v]x (只yaw), 0, 0]
        float val0 = 0;
        val0 += row_P[2] * R_inv[0*3+0] + row_P[3] * R_inv[0*3+1]; // P*H_vel^T (vel at 2,3, 只x,y)
        val0 += row_P[4] * H_theta[0*3+2]; // P*H_att^T (yaw at 4, 只z分量)
        PHt[r*3 + 0] = val0;

        // --- Col 1 (Meas Vy) ---
        // H = [0, R^T[1], R^T[1]*[v]x (只yaw), 0, 0]
        float val1 = 0;
        val1 += row_P[2] * R_inv[1*3+0] + row_P[3] * R_inv[1*3+1]; 
        val1 += row_P[4] * H_theta[1*3+2]; // 只yaw分量
        PHt[r*3 + 1] = val1;
        
        // --- Col 2 (Meas Wz) ---
        // 观测方程 h(x) = imu_wz - bg_z
        // H = d(h(x))/dx. h(x) = imu - bg. H_bg = -1.
        // So PHt = P * H^T = P * (-1) = -P_col_10 (bg_z at index 10)
        PHt[r*3 + 2] = -row_P[10]; 
    }

    // S = H * PHt + Noise (3x3)
    float S[9] = {0}; 
    
    // 填充 S 的前 2x2 (速度部分)
    // Row 0,1 of H corresponds to Vx, Vy
    for(int m_row = 0; m_row < 2; m_row++) {
        for(int m_col = 0; m_col < 3; m_col++) {
             // S[r, c] = H[r] * PHt[:, c]
             // H[r] has part R_inv[r] at Vel(2,3) and H_theta[r][2] at Att(4)
             float val = 0;
             val += R_inv[m_row*3+0] * PHt[2*3 + m_col];
             val += R_inv[m_row*3+1] * PHt[3*3 + m_col];
             
             val += H_theta[m_row*3+2] * PHt[4*3 + m_col]; // 只yaw分量
             S[m_row*3 + m_col] = val;
        }
    }
    
    // 填充 S 的第 2 行 (Wz部分)
    // dim 2 的 H 只有 H[10] = -1, 其他为0
    for(int m_col = 0; m_col < 3; m_col++) {
        // S[2, c] = H[2] * PHt[:, c] = -1.0 * PHt[10, c]
        S[2*3 + m_col] = -PHt[10*3 + m_col];
    }
    // Add noise R
    S[0*3+0] += eskf->config.odom_vel_x_noise;
    S[1*3+1] += eskf->config.odom_vel_y_noise;
    S[2*3+2] += eskf->config.odom_wz_noise;

    // S^-1
    float Si[9];
    if (mat_inv(S, Si, 3) != 0) return;

    // K = PHt * Si (11x3 * 3x3 = 11x3)
    float K[33]; // 11*3
    mat_mul(PHt, Si, K, 11, 3, 3);

    // dx = K * Y (11x3 * 3x1)
    float dx[11];
    mat_mul(K, Y, dx, 11, 3, 1);
    
    // Update P = P - K * PHt^T
    // P (11x11) -= K(11x3) * PHt^T(3x11)
    for(int r=0; r<11; r++) {
        for(int c=0; c<11; c++) {
            float corr = 0.0f;
            for(int k=0; k<3; k++) {
                corr += K[r*3+k] * PHt[c*3+k];
            }
            P[r*11+c] -= corr;
        }
    }
    
    // 状态注入
    X_curr.pos.x += dx[0]; X_curr.pos.y += dx[1]; 
    // X_curr.pos.z 保持不变
    X_curr.vel.x += dx[2]; X_curr.vel.y += dx[3]; 
    // X_curr.vel.z 保持不变
    
    // 只更新yaw，保持roll和pitch固定
    if (fabs(dx[4]) > 1e-6f) {
        float h_angle = dx[4] * 0.5f;
        eskf_quat_t dq = {cosf(h_angle), 0, 0, sinf(h_angle)};
        X_curr.rot = quat_mul(X_curr.rot, dq); 
        quat_normalize(&X_curr.rot);
    }
    X_curr.ba.x += dx[5];  X_curr.ba.y += dx[6]; X_curr.ba.z += dx[7];
    X_curr.bg.x += dx[8]; X_curr.bg.y += dx[9]; X_curr.bg.z += dx[10];

    // 重积分
    eskf->X_buf[state_idx] = X_curr;
    eskf_t temp_eskf;
    temp_eskf.config = eskf->config;
    temp_eskf.X = X_curr;
    temp_eskf.P = *P_ptr;
    
    int curr_idx = state_idx;
    int next_idx = (state_idx + 1) % ESKF_BUF_SIZE;
    int loop_guard = 0;
    while (next_idx != eskf->buf_head && loop_guard < ESKF_BUF_SIZE) {
        float dt_segment = eskf->t_buf[next_idx] - eskf->t_buf[curr_idx];
        if (dt_segment <= 1e-4f) dt_segment = 0.01f; 
        const eskf_imu_meas_t *meas_seg = &eskf->imu_buf[next_idx];
        propagate_one_step(&temp_eskf, meas_seg, dt_segment);
        
        // 检查该时刻是否有有效的里程计数据
        // 使用最近邻匹配，时间差超过阈值则跳过
        if (eskf->odom_valid[next_idx]) {
            float time_diff = fabs(eskf->odom_timestamp_buf[next_idx] - eskf->t_buf[next_idx]);
            // 时间差阈值：50ms，超过则跳过更新
            if (time_diff < 0.05f) {
                eskf_update_odom_internal(&temp_eskf, &eskf->odom_buf[next_idx], meas_seg);
            }
        }
        
        eskf->X_buf[next_idx] = temp_eskf.X;
        eskf->P_buf[next_idx] = temp_eskf.P;
        curr_idx = next_idx;
        next_idx = (next_idx + 1) % ESKF_BUF_SIZE;
        loop_guard++;
    }
    eskf->X = temp_eskf.X;
    eskf->P = temp_eskf.P;
    eskf->last_success_update_time = meas->timestamp;
}

// ------------------------------------------------------------------------
// 内部里程计更新函数 (用于重积分过程中)
// ------------------------------------------------------------------------
void eskf_update_odom_internal(eskf_t *eskf, const eskf_odom_meas_t *meas, const eskf_imu_meas_t *imu_meas) {
    if (!eskf->is_initialized) return;

    eskf_state_t X_curr = eskf->X;
    eskf_cov_t   *P_ptr = &eskf->P;
    float *P = P_ptr->P;

    // 1. 准备数据
    // R: Rotation from Body to World (3x3)
    float R[9];
    quat_to_rotmat(X_curr.rot, R);
    
    // v_w: World Velocity
    float v_w[3] = {X_curr.vel.x, X_curr.vel.y, X_curr.vel.z};

    // 使用传入的 IMU 数据 (用于计算角速度残差)
    if (imu_meas == NULL) {
        // 如果没有传入 IMU 数据，使用 last_gyro 和 last_acc
        eskf_imu_meas_t temp_imu;
        temp_imu.gyro = eskf->last_gyro;
        temp_imu.acc = eskf->last_acc;
        imu_meas = &temp_imu;
    }

    // 2. 准备观测 (Predict Measurement)
    // v_b_pred = R^T * v_w
    // wz_pred  = gyro_z - bg_z
    float v_b_pred[3];
    float R_inv[9]; // R^T
    mat_trans(R, R_inv, 3, 3);
    
    // v_body = R^T * v_world
    for(int i=0; i<3; i++) {
        v_b_pred[i] = R_inv[i*3+0] * v_w[0] + R_inv[i*3+1] * v_w[1] + R_inv[i*3+2] * v_w[2];
    }

    // 3. 残差 (Residual)
    // 融合: Vx, Vy, Wz
    float Y[3];
    Y[0] = meas->v_body_x - v_b_pred[0];
    Y[1] = meas->v_body_y - v_b_pred[1];
    Y[2] = meas->v_body_wz - (imu_meas->gyro.z - X_curr.bg.z);

    // 4. 雅可比矩阵 H (3 x 12)
    // H 结构:
    // Rows 0,1 (Vel): 对 Vel 和 Att (yaw) 有非零块
    // Row  2   (Wz) : 对 Bg 有非零块 (H_wz_bg = -1, 因为 pred = gyro - bg, so d(pred)/d(bg) = -1)
    
    // 计算 H_theta = R^T * [v_w]x (只yaw分量)
    float vx_skew[9] = {
           0,   -v_w[2],  v_w[1],
         v_w[2],     0,  -v_w[0],
        -v_w[1],  v_w[0],     0
    };
    
    // H_theta = R^T * [v_w]x
    float H_theta[9]; 
    mat_mul(R_inv, vx_skew, H_theta, 3, 3, 3);
    
    // 用 H_vel 表示 R^T (2x3部分)
    // 用 H_att 表示 H_theta (2x1部分，只yaw)

    // PHt = P * H^T  (11 x 3)
    float PHt[33]; // 11 rows, 3 cols
    
    for (int r = 0; r < 11; r++) {
        float *row_P = &P[r*11];
        
        // --- Col 0 (Meas Vx) ---
        // H = [0, R^T[0], R^T[0]*[v]x (只yaw), 0, 0]
        float val0 = 0;
        val0 += row_P[2] * R_inv[0*3+0] + row_P[3] * R_inv[0*3+1]; // P*H_vel^T (vel at 2,3, 只x,y)
        val0 += row_P[4] * H_theta[0*3+2]; // P*H_att^T (yaw at 4, 只z分量)
        PHt[r*3 + 0] = val0;

        // --- Col 1 (Meas Vy) ---
        // H = [0, R^T[1], R^T[1]*[v]x (只yaw), 0, 0]
        float val1 = 0;
        val1 += row_P[2] * R_inv[1*3+0] + row_P[3] * R_inv[1*3+1]; 
        val1 += row_P[4] * H_theta[1*3+2]; // 只yaw分量
        PHt[r*3 + 1] = val1;
        
        // --- Col 2 (Meas Wz) ---
        // 观测方程 h(x) = imu_wz - bg_z
        // H = d(h(x))/dx. h(x) = imu - bg. H_bg = -1.
        // So PHt = P * H^T = P * (-1) = -P_col_10 (bg_z at index 10)
        PHt[r*3 + 2] = -row_P[10]; 
    }

    // S = H * PHt + Noise (3x3)
    float S[9] = {0}; 
    
    // 填充 S 的前 2x2 (速度部分)
    // Row 0,1 of H corresponds to Vx, Vy
    for(int m_row = 0; m_row < 2; m_row++) {
        for(int m_col = 0; m_col < 3; m_col++) {
             // S[r, c] = H[r] * PHt[:, c]
             // H[r] has part R_inv[r] at Vel(2,3) and H_theta[r][2] at Att(4)
             float val = 0;
             val += R_inv[m_row*3+0] * PHt[2*3 + m_col];
             val += R_inv[m_row*3+1] * PHt[3*3 + m_col];
             
             val += H_theta[m_row*3+2] * PHt[4*3 + m_col]; // 只yaw分量
             S[m_row*3 + m_col] = val;
        }
    }
    
    // 填充 S 的第 2 行 (Wz部分)
    // dim 2 的 H 只有 H[10] = -1, 其他为0
    for(int m_col = 0; m_col < 3; m_col++) {
        // S[2, c] = H[2] * PHt[:, c] = -1.0 * PHt[10, c]
        S[2*3 + m_col] = -PHt[10*3 + m_col];
    }
    // Add noise R
    S[0*3+0] += eskf->config.odom_vel_x_noise;
    S[1*3+1] += eskf->config.odom_vel_y_noise;
    S[2*3+2] += eskf->config.odom_wz_noise;

    // S^-1
    float Si[9];
    if (mat_inv(S, Si, 3) != 0) return;

    // K = PHt * Si (11x3 * 3x3 = 11x3)
    float K[33]; // 11*3
    mat_mul(PHt, Si, K, 11, 3, 3);

    // dx = K * Y (11x3 * 3x1)
    float dx[11];
    mat_mul(K, Y, dx, 11, 3, 1);
    
    // Update P = P - K * PHt^T
    // P (11x11) -= K(11x3) * PHt^T(3x11)
    for(int r=0; r<11; r++) {
        for(int c=0; c<11; c++) {
            float corr = 0.0f;
            for(int k=0; k<3; k++) {
                corr += K[r*3+k] * PHt[c*3+k];
            }
            P[r*11+c] -= corr;
        }
    }
    
    // 状态注入
    X_curr.pos.x += dx[0]; X_curr.pos.y += dx[1]; 
    // X_curr.pos.z 保持不变
    X_curr.vel.x += dx[2]; X_curr.vel.y += dx[3]; 
    // X_curr.vel.z 保持不变
    
    // 只更新yaw，保持roll和pitch固定
    if (fabs(dx[4]) > 1e-6f) {
        float h_angle = dx[4] * 0.5f;
        eskf_quat_t dq = {cosf(h_angle), 0, 0, sinf(h_angle)};
        X_curr.rot = quat_mul(X_curr.rot, dq); 
        quat_normalize(&X_curr.rot);
    }
    X_curr.ba.x += dx[5];  X_curr.ba.y += dx[6]; X_curr.ba.z += dx[7];
    X_curr.bg.x += dx[8]; X_curr.bg.y += dx[9]; X_curr.bg.z += dx[10];

    // 更新当前状态
    eskf->X = X_curr;
    eskf->P = *P_ptr;
}

void eskf_get_state(const eskf_t *eskf, eskf_state_t *out_state) {
    if (out_state) {
        *out_state = eskf->X;
    }
}

int eskf_is_initialized(const eskf_t *eskf) {
    return eskf && eskf->is_initialized;
}

// ==========================================
// 内部函数实现
// ==========================================

static void propagate_one_step(eskf_t *eskf, const eskf_imu_meas_t *meas, float dt) {
    eskf_state_t *X = &eskf->X;
    
    // 1. 名义状态积分
    eskf_vec3_t acc_b = vec3_sub(meas->acc, X->ba);
    eskf_vec3_t gyro_b = vec3_sub(meas->gyro, X->bg);
    
    // 只使用x和y分量进行积分，z分量保持不变
    eskf_vec3_t acc_w = quat_rotate_vec(X->rot, acc_b);
    acc_w.x -= eskf->config.gravity.x; 
    acc_w.y -= eskf->config.gravity.y;
    // 不处理z方向的加速度和速度
    
    // Pos - 只更新x和y
    eskf_vec3_t vel_dt = vec3_scale(X->vel, dt);
    eskf_vec3_t acc_dt2 = vec3_scale(acc_w, 0.5f * dt * dt);
    X->pos.x += vel_dt.x + acc_dt2.x;
    X->pos.y += vel_dt.y + acc_dt2.y;
    // X->pos.z 保持不变
    
    // Vel - 只更新x和y，保持z速度固定
    X->vel.x += acc_w.x * dt;
    X->vel.y += acc_w.y * dt;
    // X->vel.z 保持不变
    
    // Rot - 只更新yaw，保持roll和pitch固定
    // 这里简化处理，只使用z轴角速度
    float yaw_rate = gyro_b.z;
    float angle = yaw_rate * dt;
    if (fabs(angle) > 1e-6f) {
        // 只绕z轴旋转
        eskf_quat_t dq = {cosf(angle/2.0f), 0, 0, sinf(angle/2.0f)};
        X->rot = quat_mul(X->rot, dq);
        quat_normalize(&X->rot);
    }

    // 2. 误差协方差预测 (Sparse In-Place Update)
    // 新的状态顺序: pos(2), vel(2), theta(1), ba(3), bg(3)
    // F block structure:
    // F01 = dt*I (2x2)
    // F12 = -[R(am-ba)]x * dt (2x1, 只yaw分量)
    // F13 = -R * dt (2x3)
    // F24 = -R_z * dt (1x3, 只z轴分量)
    
    float *P = eskf->P.P;
    
    // Precompute common terms
    float R[9];
    {
        float q0=X->rot.w, q1=X->rot.x, q2=X->rot.y, q3=X->rot.z;
        float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;
        R[0] = q0q0+q1q1-q2q2-q3q3; R[1] = 2*(q1*q2-q0*q3);     R[2] = 2*(q1*q3+q0*q2);
        R[3] = 2*(q1*q2+q0*q3);     R[4] = q0q0-q1q1+q2q2-q3q3; R[5] = 2*(q2*q3-q0*q1);
        R[6] = 2*(q1*q3-q0*q2);     R[7] = 2*(q2*q3+q0*q1);     R[8] = q0q0-q1q1-q2q2+q3q3;
    }
    
    eskf_vec3_t ap = quat_rotate_vec(X->rot, acc_b);
    
    // 只计算与yaw相关的F12分量 (只x,y分量)
    float F12_yaw[2] = {-ap.y*dt, ap.x*dt}; // 只x,y分量
    float nRdt[9]; 
    for(int i=0; i<9; i++) nRdt[i] = -R[i] * dt;

    // Step 1: P = P * F^T (Column Update)
    for (int r = 0; r < 11; r++) {
        float *rp = &P[r*11];
        float v_vel[2] = {rp[2], rp[3]};           // vel at indices 2,3
        float v_att = rp[4];                        // yaw at index 4
        float v_ba[3]  = {rp[5], rp[6], rp[7]};     // ba at indices 5,6,7
        float v_bg[3]  = {rp[8], rp[9], rp[10]};    // bg at indices 8,9,10

        // Col 0,1 (Pos) += dt * Col 2,3 (Vel)
        rp[0] += dt * v_vel[0];
        rp[1] += dt * v_vel[1];

        // Col 2,3 (Vel) += F12^T * Col 4 (Att) + F13^T * Col 5,6,7 (Ba)
        float tmp[2] = {v_vel[0], v_vel[1]};
        for(int k=0; k<2; k++) {
            tmp[k] += v_att * F12_yaw[k]; // 只yaw分量
            for(int j=0; j<3; j++) tmp[k] += v_ba[j] * nRdt[k*3+j]; 
        }
        
        rp[2] = tmp[0]; 
        rp[3] = tmp[1]; 

        // Col 4 (Att) += F24^T * Col 8,9,10 (Bg) - 只z轴分量
        float sum = 0;
        for(int j=0; j<3; j++) sum += v_bg[j] * nRdt[8*3+j]; // 只z轴行
        rp[4] += sum;
    }

    // Step 2: P = F * P (Row Update)
    
    // Update Row 0,1 (Pos) += dt * Row 2,3 (Vel)
    for(int c=0; c<11; c++) {
        P[c] += dt * P[2*11+c];     // pos_x += dt * vel_x
        P[11+c] += dt * P[3*11+c];  // pos_y += dt * vel_y
    }

    // Update Row 2,3 (Vel) += F12 * Row 4 (Att) + F13 * Row 5,6,7 (Ba)
    for(int c=0; c<11; c++) {
        float v_att = P[4*11+c];                  // yaw
        float v_ba[3] = {P[5*11+c], P[6*11+c], P[7*11+c]}; // ba
        for(int k=0; k<2; k++) {
             float sum = 0;
             sum += F12_yaw[k] * v_att; // 只yaw分量
             for(int j=0; j<3; j++) sum += nRdt[k*3+j] * v_ba[j];
             P[(2+k)*11+c] += sum;
        }
    }

    // Update Row 4 (Att) += F24 * Row 8,9,10 (Bg)
    for(int c=0; c<11; c++) {
        float v_bg[3] = {P[8*11+c], P[9*11+c], P[10*11+c]}; // bg
        float sum = 0;
        for(int j=0; j<3; j++) sum += nRdt[8*3+j] * v_bg[j]; // 只z轴行
        P[4*11+c] += sum;
    }

    // Add Process Noise
    float dt2 = dt * dt;
    for(int i=0; i<2; i++) P[(2+i)*11 + (2+i)] += eskf->config.acc_noise * dt2;     // vel (x,y)
    P[4*11 + 4] += eskf->config.gyro_noise * dt2;                                    // yaw
    for(int i=0; i<3; i++) P[(5+i)*11 + (5+i)] += eskf->config.acc_bias_noise * dt;  // ba
    for(int i=0; i<3; i++) P[(8+i)*11 + (8+i)] += eskf->config.gyro_bias_noise * dt; // bg
}

static float quat_to_yaw(eskf_quat_t q) {
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return atan2f(siny_cosp, cosy_cosp);
}

static float normalize_angle(float angle) {
    while (angle > CONST_PI) angle -= 2.0f * CONST_PI;
    while (angle < -CONST_PI) angle += 2.0f * CONST_PI;
    return angle;
}
