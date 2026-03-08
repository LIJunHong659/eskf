#include "eskf_mat.h"

// 1. 矩阵加法 (C = A + B)
void mat_add(float *A, float *B, float *C, int rows, int cols) {
    if (!A || !B || !C) return;
    // 使用 CMSIS-DSP 的向量加法，因为矩阵在内存也是连续的 float 数组
    // 使用 arm_add_f32 效率更高，可以直接处理 rows*cols 个元素
    arm_add_f32(A, B, C, rows * cols);
}

// 2. 矩阵减法 (C = A - B)
void mat_sub(float *A, float *B, float *C, int rows, int cols) {
    if (!A || !B || !C) return;
    arm_sub_f32(A, B, C, rows * cols);
}

// 3. 矩阵乘法 (C = A * B)
// A: m x n, B: n x k -> C: m x k
void mat_mul(float *A, float *B, float *C, int m, int n, int k) {
    if (!A || !B || !C) return;
    
    arm_matrix_instance_f32 matA, matB, matC;

    // 初始化矩阵实例指向原始 buffer (无需数据拷贝)
    // 注意：CMSIS-DSP 的矩阵乘法要求 A, B, C 必须指向有效的内存区域
    arm_mat_init_f32(&matA, m, n, (float32_t *)A);
    arm_mat_init_f32(&matB, n, k, (float32_t *)B);
    arm_mat_init_f32(&matC, m, k, (float32_t *)C);
    
    // 执行乘法
    // 可能会返回 ARM_MATH_SIZE_MISMATCH (行列不匹配) 但我们这里已经控制了参数
    arm_mat_mult_f32(&matA, &matB, &matC);
}

// 4. 矩阵转置 (C = A^T)
// A: rows x cols -> C: cols x rows
void mat_trans(float *A, float *C, int rows, int cols) {
    if (!A || !C) return;
    
    arm_matrix_instance_f32 matA, matC;
    arm_mat_init_f32(&matA, rows, cols, (float32_t *)A);
    arm_mat_init_f32(&matC, cols, rows, (float32_t *)C);
    
    arm_mat_trans_f32(&matA, &matC);
}

// 5. 矩阵求逆 (B = inv(A))
// A: n x n
// 返回 0 表示成功，非0 失败
int mat_inv(float *A, float *B, int n) {
    if (!A || !B) return -1;
    
    arm_matrix_instance_f32 matA, matB;
    arm_mat_init_f32(&matA, n, n, (float32_t *)A);
    arm_mat_init_f32(&matB, n, n, (float32_t *)B);
    
    arm_status status = arm_mat_inverse_f32(&matA, &matB);
    
    return (status == ARM_MATH_SUCCESS) ? 0 : -1;
}

// 6. 单位矩阵初始化
void mat_eye(float *A, int n) {
    if (!A) return;
    for (int i = 0; i < n * n; i++) A[i] = 0.0f;
    for (int i = 0; i < n; i++) A[i * n + i] = 1.0f;
}

// 7. 零矩阵初始化
void mat_zero(float *A, int rows, int cols) {
    if (!A) return;
    // 使用 DSP 填充函数或简单 memset
    // arm_fill_f32(0.0f, A, rows * cols);
    memset(A, 0, sizeof(float) * rows * cols);
}
