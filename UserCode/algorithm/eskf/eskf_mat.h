#ifndef ESKF_MAT_H
#define ESKF_MAT_H

#include "eskf_types.h"
#include "arm_math.h" // 引用 CMSIS-DSP 库

// 封装层：让 ESKF 核心逻辑不用直接处理 arm_matrix_instance_f32
// 这样可以保持算法逻辑的纯净，底层调用 DSP 加速

// C = A + B
void mat_add(float *A, float *B, float *C, int rows, int cols);

// C = A - B
void mat_sub(float *A, float *B, float *C, int rows, int cols);

// C = A * B
// A: m x n, B: n x k -> C: m x k
void mat_mul(float *A, float *B, float *C, int m, int n, int k);

// C = A^T
// A: rows x cols -> C: cols x rows
void mat_trans(float *A, float *C, int rows, int cols);

// B = inv(A)
// A: n x n
// 返回 0 表示成功
int mat_inv(float *A, float *B, int n);

// 辅助：设置单位矩阵
void mat_eye(float *A, int n);

// 辅助：设置零矩阵
void mat_zero(float *A, int rows, int cols);

#endif // ESKF_MAT_H
