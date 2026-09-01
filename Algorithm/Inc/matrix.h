/**
 ******************************************************************************
 * @file    matrix.c/h
 * @brief   矩阵/向量运算库
 *          提供基础的矩阵和向量操作，包括初始化、四则运算、转置、求逆等功能
 *          底层基于ARM Math库实现高效的矩阵计算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include <stdbool.h>
#ifndef MATRIX_H
#define MATRIX_H

#include "arm_math.h"  // ARM数学库，提供底层矩阵运算实现

// 定义最大矩阵大小，根据实际需求调整
// 注意：增加此值会增加内存占用，减小此值可能导致大型矩阵操作失败
#define MAX_MATRIX_SIZE 100

/**
 * @brief 矩阵结构体定义
 * 
 * 此结构体用于表示和操作浮点型矩阵，采用行优先(row-major)的一维数组存储
 * 同时包含ARM Math库的矩阵实例，用于调用底层高效计算函数
 */
typedef struct {
    uint32_t rows;           // 矩阵行数
    uint32_t cols;           // 矩阵列数
    float data[MAX_MATRIX_SIZE];  // 矩阵数据（一维数组存储，行优先）
    arm_matrix_instance_f32 arm_mat;  // ARM Math库矩阵实例，用于底层计算加速
} Matrixf;

/**
 * @brief 初始化矩阵
 * 
 * @param mat 矩阵指针，指向要初始化的Matrixf结构体
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @param data 初始化数据数组（可选，NULL表示初始化为0）
 * 
 * @note 1. 如果data不为NULL，则从data数组中复制rows*cols个元素到矩阵
 *       2. 如果data为NULL，则矩阵所有元素初始化为0
 *       3. 必须确保rows*cols不超过MAX_MATRIX_SIZE
 */
void Matrixf_init(Matrixf* mat, uint32_t rows, uint32_t cols, const float* data);

/**
 * @brief 复制矩阵
 * 
 * @param dest 目标矩阵，接收复制的数据
 * @param src 源矩阵，提供要复制的数据
 * 
 * @note 1. 确保dest矩阵的大小与src矩阵相同
 *       2. 此函数不会自动调整dest矩阵的行列数，仅复制数据
 */
void Matrixf_copy(Matrixf* dest, const Matrixf* src);

/**
 * @brief 获取矩阵指定位置的元素值
 * 
 * @param mat 矩阵指针
 * @param row 行索引（从0开始）
 * @param col 列索引（从0开始）
 * @return 矩阵元素值
 * 
 * @note 行索引和列索引必须在有效范围内，否则结果未定义
 */
float Matrixf_get(const Matrixf* mat, uint32_t row, uint32_t col);

/**
 * @brief 设置矩阵指定位置的元素值
 * 
 * @param mat 矩阵指针
 * @param row 行索引（从0开始）
 * @param col 列索引（从0开始）
 * @param value 要设置的值
 * 
 * @note 行索引和列索引必须在有效范围内，否则操作无效
 */
void Matrixf_set(Matrixf* mat, uint32_t row, uint32_t col, float value);

/**
 * @brief 矩阵加法运算（result = mat1 + mat2）
 * 
 * @param result 结果矩阵，存储加法结果
 * @param mat1 第一个矩阵
 * @param mat2 第二个矩阵
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 1. 矩阵mat1和mat2必须具有相同的行列数
 *       2. 结果矩阵result的行列数必须与输入矩阵相同
 */
arm_status Matrixf_add(Matrixf* result, const Matrixf* mat1, const Matrixf* mat2);

/**
 * @brief 矩阵减法运算（result = mat1 - mat2）
 * 
 * @param result 结果矩阵，存储减法结果
 * @param mat1 第一个矩阵
 * @param mat2 第二个矩阵
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 1. 矩阵mat1和mat2必须具有相同的行列数
 *       2. 结果矩阵result的行列数必须与输入矩阵相同
 */
arm_status Matrixf_sub(Matrixf* result, const Matrixf* mat1, const Matrixf* mat2);

/**
 * @brief 矩阵乘法运算（result = mat1 × mat2）
 * 
 * @param result 结果矩阵，存储乘法结果
 * @param mat1 第一个矩阵（左矩阵）
 * @param mat2 第二个矩阵（右矩阵）
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 1. mat1的列数必须等于mat2的行数
 *       2. 结果矩阵result的行数应等于mat1的行数，列数应等于mat2的列数
 *       3. 矩阵乘法不满足交换律，mat1×mat2 ≠ mat2×mat1
 */
arm_status Matrixf_mult(Matrixf* result, const Matrixf* mat1, const Matrixf* mat2);

/**
 * @brief 矩阵数乘运算（result = scalar × mat）
 * 
 * @param result 结果矩阵，存储数乘结果
 * @param mat 输入矩阵
 * @param scalar 标量因子
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 结果矩阵result的行列数必须与输入矩阵相同
 */
arm_status Matrixf_scale(Matrixf* result, const Matrixf* mat, float scalar);

/**
 * @brief 矩阵转置运算（result = mat^T）
 * 
 * @param result 结果矩阵，存储转置后的矩阵
 * @param mat 源矩阵
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 1. 转置操作会将矩阵的行和列互换
 *       2. 结果矩阵result的行数应等于源矩阵的列数，列数应等于源矩阵的行数
 */
arm_status Matrixf_trans(Matrixf* result, const Matrixf* mat);

/**
 * @brief 矩阵求逆运算（result = mat^(-1)）
 * 
 * @param result 结果矩阵，存储矩阵的逆
 * @param mat 源矩阵
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败（如奇异矩阵）
 * 
 * @note 1. 仅方阵（行数=列数）可以求逆
 *       2. 矩阵必须是非奇异的（行列式不为零）
 *       3. 结果矩阵result的行列数必须与源矩阵相同
 */
arm_status Matrixf_inv(Matrixf* result, const Matrixf* mat);

/**
 * @brief 创建零矩阵（所有元素均为0）
 * 
 * @param mat 矩阵指针，将被初始化为零矩阵
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * 
 * @note 必须确保rows*cols不超过MAX_MATRIX_SIZE
 */
void Matrixf_zeros(Matrixf* mat, uint32_t rows, uint32_t cols);

/**
 * @brief 创建全1矩阵（所有元素均为1）
 * 
 * @param mat 矩阵指针，将被初始化为全1矩阵
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * 
 * @note 必须确保rows*cols不超过MAX_MATRIX_SIZE
 */
void Matrixf_ones(Matrixf* mat, uint32_t rows, uint32_t cols);

/**
 * @brief 创建单位矩阵（主对角线元素为1，其他元素为0）
 * 
 * @param mat 矩阵指针，将被初始化为单位矩阵
 * @param size 矩阵大小（行数=列数）
 * 
 * @note 1. 单位矩阵是一个方阵
 *       2. 必须确保size*size不超过MAX_MATRIX_SIZE
 */
void Matrixf_eye(Matrixf* mat, uint32_t size);

/**
 * @brief 创建对角矩阵（对角线上的元素由vec提供，其他元素为0）
 * 
 * @param mat 矩阵指针，将被初始化为对角矩阵
 * @param vec 包含对角线元素的一维数组
 * @param size 矩阵大小（行数=列数）
 * 
 * @note 1. 对角矩阵是一个方阵
 *       2. 必须确保size*size不超过MAX_MATRIX_SIZE
 */
void Matrixf_diag(Matrixf* mat, const float* vec, uint32_t size);

/**
 * @brief 计算矩阵的迹（主对角线元素之和）
 * 
 * @param mat 矩阵指针
 * @return 矩阵的迹
 * 
 * @note 仅方阵（行数=列数）有迹
 */
float Matrixf_trace(const Matrixf* mat);

/**
 * @brief 计算矩阵的Frobenius范数（元素平方和的平方根）
 * 
 * @param mat 矩阵指针
 * @return 矩阵的Frobenius范数
 * 
 * @note Frobenius范数是向量欧几里得范数在矩阵上的推广
 *       计算公式：||A||_F = sqrt(Σ|a_ij|2)
 */
float Matrixf_norm(const Matrixf* mat);

/**
 * @brief 比较两个矩阵是否相等
 * 
 * @param mat1 第一个矩阵
 * @param mat2 第二个矩阵
 * @return 如果两个矩阵大小相同且所有对应元素相等，则返回true；否则返回false
 * 
 * @note 使用浮点数比较时会考虑一定的精度误差
 */
bool Matrixf_equal(const Matrixf* mat1, const Matrixf* mat2);

/**
 * @brief 从源矩阵中提取子矩阵
 * 
 * @param result 结果矩阵，存储提取的子矩阵
 * @param src 源矩阵
 * @param start_row 子矩阵在源矩阵中的起始行索引
 * @param start_col 子矩阵在源矩阵中的起始列索引
 * @param rows 子矩阵的行数
 * @param cols 子矩阵的列数
 * 
 * @note 1. 确保提取的子矩阵在源矩阵的有效范围内
 *       2. 结果矩阵的大小应设置为rows×cols
 */
void Matrixf_block(Matrixf* result, const Matrixf* src, uint32_t start_row, uint32_t start_col, uint32_t rows, uint32_t cols);

/**
 * @brief 提取矩阵的一行作为向量
 * 
 * @param result 结果向量（1×cols的矩阵），存储提取的行
 * @param src 源矩阵
 * @param row 要提取的行索引
 * 
 * @note 结果矩阵应预先初始化为1×cols的矩阵
 */
void Matrixf_row(Matrixf* result, const Matrixf* src, uint32_t row);

/**
 * @brief 提取矩阵的一列作为向量
 * 
 * @param result 结果向量（rows×1的矩阵），存储提取的列
 * @param src 源矩阵
 * @param col 要提取的列索引
 * 
 * @note 结果矩阵应预先初始化为rows×1的矩阵
 */
void Matrixf_col(Matrixf* result, const Matrixf* src, uint32_t col);

#endif /* MATRIX_H */
