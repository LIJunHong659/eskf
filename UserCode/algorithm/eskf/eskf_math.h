#ifndef ESKF_MATH_H
#define ESKF_MATH_H

#include "eskf_types.h"
#include <math.h>

// 向量加法
static inline eskf_vec3_t vec3_add(eskf_vec3_t a, eskf_vec3_t b) {
    return (eskf_vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

// 向量减法
static inline eskf_vec3_t vec3_sub(eskf_vec3_t a, eskf_vec3_t b) {
    return (eskf_vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

// 向量数乘
static inline eskf_vec3_t vec3_scale(eskf_vec3_t a, eskf_float_t s) {
    return (eskf_vec3_t){a.x * s, a.y * s, a.z * s};
}

// 向量点乘
static inline eskf_float_t vec3_dot(eskf_vec3_t a, eskf_vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 向量叉乘
static inline eskf_vec3_t vec3_cross(eskf_vec3_t a, eskf_vec3_t b) {
    return (eskf_vec3_t){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

// 向量模长
static inline eskf_float_t vec3_norm(eskf_vec3_t a) {
    return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
}

// 四元数乘法
static inline eskf_quat_t quat_mul(eskf_quat_t q, eskf_quat_t p) {
    return (eskf_quat_t){
        q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
        q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
        q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
        q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
    };
}

// 四元数旋转向量 (q * v * q_inv)
static inline eskf_vec3_t quat_rotate_vec(eskf_quat_t q, eskf_vec3_t v) {
    // 简化实现: t = 2 * cross(q.xyz, v)
    // v' = v + q.w * t + cross(q.xyz, t)
    eskf_float_t tx = 2.0f * (q.y * v.z - q.z * v.y);
    eskf_float_t ty = 2.0f * (q.z * v.x - q.x * v.z);
    eskf_float_t tz = 2.0f * (q.x * v.y - q.y * v.x);
    
    return (eskf_vec3_t){
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx)
    };
}

// 四元数归一化
static inline void quat_normalize(eskf_quat_t *q) {
    eskf_float_t n = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (n > 1e-6f) {
        q->w /= n; q->x /= n; q->y /= n; q->z /= n;
    }
}

// 矩阵运算通常比较复杂，这里仅作声明，实际项目建议使用 CMSIS-DSP
// 假设有如下函数：
// void mat_mul(const float *A, const float *B, float *C, int m, int n, int k); // C = A * B
// void mat_inv(const float *A, float *B, int n); // B = inv(A)
// void mat_add(const float *A, const float *B, float *C, int n); // C = A + B
// void mat_sub(const float *A, const float *B, float *C, int n); // C = A - B

#endif // ESKF_MATH_H
