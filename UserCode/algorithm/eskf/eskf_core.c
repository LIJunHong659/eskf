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
    
    memset(eskf, 0, sizeof(eskf_t));
    eskf->config = *config;
    
    // 初始化状态
    eskf->X.rot.w = 1.0f; // Unit Quaternion
    
    // 初始化协方差 P0 (对角阵)
    mat_zero(eskf->P.P, ESKF_STATE_DIM, ESKF_STATE_DIM);
    for (int i = 0; i < ESKF_STATE_DIM; i++) {
        float var;
        if (i < 3) var = config->init_pos_unc;
        else if (i < 6) var = config->init_vel_unc;
        else if (i < 9) var = config->init_att_unc;
        else var = config->init_bias_unc;
        
        eskf->P.P[i * 15 + i] = var * var;
    }

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

    int best_idx = -1;
    double min_diff = 1e9;
    
    int idx = eskf->buf_tail;
    for (int i = 0; i < eskf->buf_count; i++) {
        double diff = fabs(eskf->t_buf[idx] - t_target);
        if (diff < min_diff) {
            min_diff = diff;
            best_idx = idx;
        }
        idx = (idx + 1) % ESKF_BUF_SIZE;
    }
    
    if (min_diff > 0.1) return -1; // Too old or too new
    
    return best_idx;
}

void eskf_update_pos(eskf_t *eskf, const eskf_pos_meas_t *meas) {
    if (!eskf->is_initialized) {
        eskf->X.pos.x = meas->x;
        eskf->X.pos.y = meas->y;
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

    // H 矩阵稀疏特性: H_pos 选择 Pos(0,1), H_yaw 选择 ThetaZ(8)
    int idx_map[3] = {0, 1, 8}; 

    // PHt = P * H^T (Extract columns 0, 1, [8] of P)
    float PHt[15 * 3]; 
    for (int c = 0; c < meas_dim; c++) {
        int state_col = idx_map[c];
        for (int r = 0; r < 15; r++) {
            PHt[r * meas_dim + c] = P[r * 15 + state_col];
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
    float K[15 * 3];
    mat_mul(PHt, Si, K, 15, meas_dim, meas_dim);
    
    // dx = K * Y
    float dx[15];
    mat_mul(K, Y, dx, 15, meas_dim, 1);
    
    // Update P = P - K * (H P)
    // HP (meas_dim x 15) is Transpose of PHt (symmetric P)
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            float correction = 0.0f;
            for (int k = 0; k < meas_dim; k++) {
                // K[i][k] * HP[k][j] -> HP[k][j] is P[idx_map[k]][j]
                correction += K[i * meas_dim + k] * P[idx_map[k] * 15 + j];
            }
            P[i * 15 + j] -= correction;
        }
    }

    // 5. 误差状态注入
    X_curr.pos.x += dx[0];
    X_curr.pos.y += dx[1];
    X_curr.pos.z += dx[2];
    X_curr.vel.x += dx[3];
    X_curr.vel.y += dx[4];
    X_curr.vel.z += dx[5];
    
    eskf_vec3_t theta_err = {dx[6], dx[7], dx[8]};
    float theta_norm = vec3_norm(theta_err);
    if (theta_norm > 1e-6f) {
        float h_angle = theta_norm * 0.5f;
        float s = sinf(h_angle) / theta_norm;
        eskf_quat_t dq = {cosf(h_angle), dx[6]*s, dx[7]*s, dx[8]*s};
        X_curr.rot = quat_mul(X_curr.rot, dq); 
        quat_normalize(&X_curr.rot);
    }
    X_curr.ba.x += dx[9];  X_curr.ba.y += dx[10]; X_curr.ba.z += dx[11];
    X_curr.bg.x += dx[12]; X_curr.bg.y += dx[13]; X_curr.bg.z += dx[14];

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

    // 5. 雅可比矩阵 H (3 x 15)
    // H 结构:
    // Rows 0,1 (Vel): 对 Vel 和 Att 有非零块
    // Row  2   (Wz) : 对 Bg 有非零块 (H_wz_bg = -1, 因为 pred = gyro - bg, so d(pred)/d(bg) = -1)
    
    // 计算 H_theta = R^T * [v_w]x
    float vx_skew[9] = {
           0,   -v_w[2],  v_w[1],
         v_w[2],     0,  -v_w[0],
        -v_w[1],  v_w[0],     0
    };
    
    // H_theta = R^T * [v_w]x
    float H_theta[9]; 
    mat_mul(R_inv, vx_skew, H_theta, 3, 3, 3);
    
    // 用 H_vel 表示 R^T (2x3部分)
    // 用 H_att 表示 H_theta (2x3部分)

    // PHt = P * H^T  (15 x 3)
    float PHt[45]; // 15 rows, 3 cols
    
    for (int r = 0; r < 15; r++) {
        float *row_P = &P[r*15];
        
        // --- Col 0 (Meas Vx) ---
        // H = [0, R^T[0], R^T[0]*[v]x, 0, 0]
        float val0 = 0;
        val0 += row_P[3] * R_inv[0*3+0] + row_P[4] * R_inv[0*3+1] + row_P[5] * R_inv[0*3+2]; // P*H_vel^T
        val0 += row_P[6] * H_theta[0*3+0] + row_P[7] * H_theta[0*3+1] + row_P[8] * H_theta[0*3+2]; // P*H_att^T
        PHt[r*3 + 0] = val0;

        // --- Col 1 (Meas Vy) ---
        // H = [0, R^T[1], R^T[1]*[v]x, 0, 0]
        float val1 = 0;
        val1 += row_P[3] * R_inv[1*3+0] + row_P[4] * R_inv[1*3+1] + row_P[5] * R_inv[1*3+2]; 
        val1 += row_P[6] * H_theta[1*3+0] + row_P[7] * H_theta[1*3+1] + row_P[8] * H_theta[1*3+2];
        PHt[r*3 + 1] = val1;
        
        // --- Col 2 (Meas Wz) ---
        // 观测方程 h(x) = imu_wz - bg_z
        // H = d(y)/dx = d(meas - h(x))/dx = d(- (-bg_z))/dx = d(bg_z)/dx = 1 ????
        // 等一下，Residual Y = Meas - Pred.
        // Pred = imu - bg.
        // Y = meas - (imu - bg) = meas - imu + bg.
        // dY/dbg = 1.
        // 也就是说，H 在 bg_z (index 14) 处为 1. (之前推导以为是-1, 这里必须再次确认)
        // 状态更新: x += K * Y.
        // 如果测量的 Wz > 预测的 (imu-bg), 说明 Y > 0.
        // 意味着 "真实的Wz" 比 "预测的" 大。
        // IMU测量值是固定的。意味着 bg 可能估计大了(减多了)，或者 bg 估计小了？
        // 如果 Y > 0 => meas > imu - bg => meas - imu > -bg => bg > imu - meas.
        // 这表示我们需要增加 bg。
        // 如果 H=1, K 正比于 P*H^T = P_col_bg.
        // x += K * Y => bg += P_bb * 1 * Y. 正相关。
        // 所以 H 对应 bg_z 应该是 1.
        
        // H = [0, ..., 0, -1 (at idx 14)] (d(meas - (imu - bg))/dbg = 1) -> No.
        // H = d(h(x))/dx. h(x) = imu - bg. H_bg = -1.
        // So PHt = P * H^T = P * (-1) = -P_col_14.
        PHt[r*3 + 2] = -row_P[14]; 
    }

    // S = H * PHt + Noise (3x3)
    float S[9] = {0}; 
    
    // 填充 S 的前 2x2 (速度部分)
    // Row 0,1 of H corresponds to Vx, Vy
    for(int m_row = 0; m_row < 2; m_row++) {
        for(int m_col = 0; m_col < 3; m_col++) {
             // S[r, c] = H[r] * PHt[:, c]
             // H[r] has part R_inv[r] at Vel(3,4,5) and H_theta[r] at Att(6,7,8)
             float val = 0;
             val += R_inv[m_row*3+0] * PHt[3*3 + m_col];
             val += R_inv[m_row*3+1] * PHt[4*3 + m_col];
             val += R_inv[m_row*3+2] * PHt[5*3 + m_col];
             
             val += H_theta[m_row*3+0] * PHt[6*3 + m_col];
             val += H_theta[m_row*3+1] * PHt[7*3 + m_col];
             val += H_theta[m_row*3+2] * PHt[8*3 + m_col];
             S[m_row*3 + m_col] = val;
        }
    }
    
    // 填充 S 的第 2 行 (Wz部分)
    // dim 2 的 H 只有 H[14] = -1, 其他为0
    for(int m_col = 0; m_col < 3; m_col++) {
        // S[2, c] = H[2] * PHt[:, c] = -1.0 * PHt[14, c]
        S[2*3 + m_col] = -PHt[14*3 + m_col];
    }
    // Add noise R
    S[0*3+0] += eskf->config.odom_vel_x_noise;
    S[1*3+1] += eskf->config.odom_vel_y_noise;
    S[2*3+2] += eskf->config.odom_wz_noise;

    // S^-1
    float Si[9];
    if (mat_inv(S, Si, 3) != 0) return;

    // K = PHt * Si (15x3 * 3x3 = 15x3)
    float K[45]; // 15*3
    mat_mul(PHt, Si, K, 15, 3, 3);

    // dx = K * Y (15x3 * 3x1)
    float dx[15];
    mat_mul(K, Y, dx, 15, 3, 1);
    
    // Update P = P - K * PHt^T
    // P (15x15) -= K(15x3) * PHt^T(3x15)
    for(int r=0; r<15; r++) {
        for(int c=0; c<15; c++) {
            float corr = 0.0f;
            for(int k=0; k<3; k++) {
                corr += K[r*3+k] * PHt[c*3+k];
            }
            P[r*15+c] -= corr;
        }
    }
    
    // 状态注入
    X_curr.pos.x += dx[0]; X_curr.pos.y += dx[1]; X_curr.pos.z += dx[2];
    X_curr.vel.x += dx[3]; X_curr.vel.y += dx[4]; X_curr.vel.z += dx[5];
    
    eskf_vec3_t theta_err = {dx[6], dx[7], dx[8]};
    float theta_norm = vec3_norm(theta_err);
    if (theta_norm > 1e-6f) {
        float h_angle = theta_norm * 0.5f;
        float s = sinf(h_angle) / theta_norm;
        eskf_quat_t dq = {cosf(h_angle), dx[6]*s, dx[7]*s, dx[8]*s};
        X_curr.rot = quat_mul(X_curr.rot, dq); 
        quat_normalize(&X_curr.rot);
    }
    X_curr.ba.x += dx[9];  X_curr.ba.y += dx[10]; X_curr.ba.z += dx[11];
    X_curr.bg.x += dx[12]; X_curr.bg.y += dx[13]; X_curr.bg.z += dx[14];

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
    
    eskf_vec3_t acc_w = quat_rotate_vec(X->rot, acc_b);
    acc_w.x -= eskf->config.gravity.x; 
    acc_w.y -= eskf->config.gravity.y;
    acc_w.z -= eskf->config.gravity.z; 
    
    // Pos
    X->pos = vec3_add(X->pos, vec3_add(vec3_scale(X->vel, dt), vec3_scale(acc_w, 0.5f * dt * dt)));
    // Vel
    X->vel = vec3_add(X->vel, vec3_scale(acc_w, dt));
    
    // Rot (0th order)
    float angle = vec3_norm(gyro_b) * dt;
    if (angle > 1e-6f) {
        eskf_vec3_t axis = vec3_scale(gyro_b, dt / angle);
        float s = sinf(angle / 2.0f);
        float c = cosf(angle / 2.0f);
        eskf_quat_t dq = {c, axis.x * s, axis.y * s, axis.z * s};
        X->rot = quat_mul(X->rot, dq);
        quat_normalize(&X->rot);
    }

    // 2. 误差协方差预测 (Sparse In-Place Update)
    // F block structure:
    // F01 = dt*I
    // F12 = -[R(am-ba)]x * dt
    // F13 = F24 = -R * dt
    
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
    float F12[9] = {0, ap.z*dt, -ap.y*dt, -ap.z*dt, 0, ap.x*dt, ap.y*dt, -ap.x*dt, 0};
    float nRdt[9]; 
    for(int i=0; i<9; i++) nRdt[i] = -R[i] * dt;

    // Step 1: P = P * F^T (Column Update)
    for (int r = 0; r < 15; r++) {
        float *rp = &P[r*15];
        float v_vel[3] = {rp[3], rp[4], rp[5]};
        float v_att[3] = {rp[6], rp[7], rp[8]};
        float v_ba[3]  = {rp[9], rp[10], rp[11]};
        float v_bg[3]  = {rp[12], rp[13], rp[14]};

        // Col 0 (Pos) += dt * Col 1
        rp[0] += dt * v_vel[0]; rp[1] += dt * v_vel[1]; rp[2] += dt * v_vel[2];

        // Col 1 (Vel) += F12^T * Col 2 + F13^T * Col 3
        float tmp[3] = {v_vel[0], v_vel[1], v_vel[2]};
        for(int k=0; k<3; k++) {
            for(int j=0; j<3; j++) tmp[k] += v_att[j] * F12[k*3+j] + v_ba[j] * nRdt[k*3+j]; 
        }
        
        rp[3]=tmp[0]; rp[4]=tmp[1]; rp[5]=tmp[2];

        // Col 2 (Att) += F24^T * Col 4
        for(int k=0; k<3; k++) {
            float sum = 0;
            for(int j=0; j<3; j++) sum += v_bg[j] * nRdt[k*3+j];
            rp[6+k] += sum;
        }
    }

    // Step 2: P = F * P (Row Update)
    // Update Row 0, 1, 2 sequentially.
    
    // Update Row 0 (Pos) += dt * Row 1
    for(int c=0; c<15; c++) {
        P[c] += dt * P[3*15+c];
        P[15+c] += dt * P[4*15+c];
        P[30+c] += dt * P[5*15+c];
    }

    // Update Row 1 (Vel) += F12 * Row 2 + F13 * Row 3
    for(int c=0; c<15; c++) {
        float v_att[3] = {P[6*15+c], P[7*15+c], P[8*15+c]};
        float v_ba[3]  = {P[9*15+c], P[10*15+c], P[11*15+c]};
        for(int k=0; k<3; k++) {
             float sum = 0;
             for(int j=0; j<3; j++) sum += F12[k*3+j] * v_att[j] + nRdt[k*3+j] * v_ba[j];
             P[(3+k)*15+c] += sum;
        }
    }

    // Update Row 2 (Att) += F24 * Row 4
    for(int c=0; c<15; c++) {
        float v_bg[3] = {P[12*15+c], P[13*15+c], P[14*15+c]};
        for(int k=0; k<3; k++) {
             float sum = 0;
             for(int j=0; j<3; j++) sum += nRdt[k*3+j] * v_bg[j];
             P[(6+k)*15+c] += sum;
        }
    }

    // Add Process Noise
    float dt2 = dt * dt;
    for(int i=0; i<3; i++) P[(3+i)*15 + (3+i)] += eskf->config.acc_noise * dt2;
    for(int i=0; i<3; i++) P[(6+i)*15 + (6+i)] += eskf->config.gyro_noise * dt2; 
    for(int i=0; i<3; i++) P[(9+i)*15 + (9+i)] += eskf->config.acc_bias_noise * dt;
    for(int i=0; i<3; i++) P[(12+i)*15 + (12+i)] += eskf->config.gyro_bias_noise * dt;
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
