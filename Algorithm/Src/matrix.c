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

#include "Matrix.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
 *       4. 初始化完成后，设置ARM Math库矩阵实例，以便后续调用底层加速函数
 */
void Matrixf_init(Matrixf* mat, uint32_t rows, uint32_t cols, const float* data) {
    if (mat == NULL || rows * cols > MAX_MATRIX_SIZE) {
        return;
    }
    
    mat->rows = rows;
    mat->cols = cols;
    
    if (data != NULL) {
        memcpy(mat->data, data, rows * cols * sizeof(float));
    } else {
        memset(mat->data, 0, rows * cols * sizeof(float));
    }
    
//		 mat->arm_mat.numRows = (uint16_t)rows;   // 注意类型转换
//    mat->arm_mat.numCols = (uint16_t)cols;   // ARM使用uint16_t
//    mat->arm_mat.pData = (float32_t*)data;               // 指向同一数据
    arm_mat_init_f32(&mat->arm_mat, rows, cols, mat->data);
}

/**
 * @brief 复制矩阵
 * 
 * @param dest 目标矩阵，接收复制的数据
 * @param src 源矩阵，提供要复制的数据
 * 
 * @note 1. 确保dest矩阵的大小与src矩阵相同
 *       2. 此函数通过调用Matrixf_init实现数据复制，同时更新ARM Math库矩阵实例
 */
void Matrixf_copy(Matrixf* dest, const Matrixf* src) {
    if (dest == NULL || src == NULL) {
        return;
    }
    
    Matrixf_init(dest, src->rows, src->cols, src->data);
}

/**
 * @brief 获取矩阵指定位置的元素值
 * 
 * @param mat 矩阵指针
 * @param row 行索引（从0开始）
 * @param col 列索引（从0开始）
 * @return 矩阵元素值，如果索引越界或mat为NULL则返回0
 * 
 * @note 行索引和列索引必须在有效范围内，否则结果未定义
 *       内部使用行优先存储，索引计算方式为：row * cols + col
 */
float Matrixf_get(const Matrixf* mat, uint32_t row, uint32_t col) {
    if (mat == NULL || row >= mat->rows || col >= mat->cols) {
        return 0.0f;
    }
    
    return mat->data[row * mat->cols + col];
}

/**
 * @brief 设置矩阵指定位置的元素值
 * 
 * @param mat 矩阵指针
 * @param row 行索引（从0开始）
 * @param col 列索引（从0开始）
 * @param value 要设置的值
 * 
 * @note 1. 行索引和列索引必须在有效范围内，否则操作无效
 *       2. 内部使用行优先存储，索引计算方式为：row * cols + col
 */
void Matrixf_set(Matrixf* mat, uint32_t row, uint32_t col, float value) {
    if (mat == NULL || row >= mat->rows || col >= mat->cols) {
        return;
    }
    
    mat->data[row * mat->cols + col] = value;
}

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
 *       3. 使用ARM Math库的arm_mat_add_f32函数实现底层计算
 */
arm_status Matrixf_add(Matrixf* result, const Matrixf* mat1, const Matrixf* mat2) {
    if (result == NULL || mat1 == NULL || mat2 == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (mat1->rows != mat2->rows || mat1->cols != mat2->cols) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    
    Matrixf_init(result, mat1->rows, mat1->cols, NULL);
    return arm_mat_add_f32(&mat1->arm_mat, &mat2->arm_mat, &result->arm_mat);
}

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
 *       3. 使用ARM Math库的arm_mat_sub_f32函数实现底层计算
 */
arm_status Matrixf_sub(Matrixf* result, const Matrixf* mat1, const Matrixf* mat2) {
    if (result == NULL || mat1 == NULL || mat2 == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (mat1->rows != mat2->rows || mat1->cols != mat2->cols) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    
    Matrixf_init(result, mat1->rows, mat1->cols, NULL);
    return arm_mat_sub_f32(&mat1->arm_mat, &mat2->arm_mat, &result->arm_mat);
}

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
 *       4. 使用ARM Math库的arm_mat_mult_f32函数实现底层计算
 */
arm_status Matrixf_mult(Matrixf* result, const Matrixf* mat1, const Matrixf* mat2) {
    if (result == NULL || mat1 == NULL || mat2 == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (mat1->cols != mat2->rows) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    
    Matrixf_init(result, mat1->rows, mat2->cols, NULL);
    return arm_mat_mult_f32(&mat1->arm_mat, &mat2->arm_mat, &result->arm_mat);
}

/**
 * @brief 矩阵数乘运算（result = scalar × mat）
 * 
 * @param result 结果矩阵，存储数乘结果
 * @param mat 输入矩阵
 * @param scalar 标量因子
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 1. 结果矩阵result的行列数必须与输入矩阵相同
 *       2. 使用ARM Math库的arm_mat_scale_f32函数实现底层计算
 */
arm_status Matrixf_scale(Matrixf* result, const Matrixf* mat, float scalar) {
    if (result == NULL || mat == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    Matrixf_init(result, mat->rows, mat->cols, NULL);
		for (uint32_t i = 0; i < mat->rows * mat->cols; i++) {
        result->data[i] = mat->data[i]*scalar;
    }
    return arm_mat_scale_f32(&mat->arm_mat, scalar, &result->arm_mat);
}

/**
 * @brief 矩阵转置运算（result = mat^T）
 * 
 * @param result 结果矩阵，存储转置后的矩阵
 * @param mat 源矩阵
 * @return 操作状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 * 
 * @note 1. 转置操作会将矩阵的行和列互换
 *       2. 结果矩阵result的行数应等于源矩阵的列数，列数应等于源矩阵的行数
 *       3. 使用ARM Math库的arm_mat_trans_f32函数实现底层计算
 */
arm_status Matrixf_trans(Matrixf* result, const Matrixf* mat) {
    if (result == NULL || mat == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    Matrixf_init(result, mat->cols, mat->rows, NULL);
    return arm_mat_trans_f32(&mat->arm_mat, &result->arm_mat);
}

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
 *       4. 使用ARM Math库的arm_mat_inverse_f32函数实现底层计算
 */
arm_status Matrixf_inv(Matrixf* result, const Matrixf* mat) {
    if (result == NULL || mat == NULL) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (mat->rows != mat->cols) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    
    Matrixf_init(result, mat->rows, mat->cols, NULL);
    return arm_mat_inverse_f32(&mat->arm_mat, &result->arm_mat);
}

/**
 * @brief 创建零矩阵（所有元素均为0）
 * 
 * @param mat 矩阵指针，将被初始化为零矩阵
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * 
 * @note 直接调用Matrixf_init函数，传入NULL作为数据参数即可创建零矩阵
 */
void Matrixf_zeros(Matrixf* mat, uint32_t rows, uint32_t cols) {
    Matrixf_init(mat, rows, cols, NULL);
}

/**
 * @brief 创建全1矩阵（所有元素均为1）
 * 
 * @param mat 矩阵指针，将被初始化为全1矩阵
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * 
 * @note 1. 首先调用Matrixf_init初始化矩阵（设为零矩阵）
 *       2. 然后遍历矩阵，将每个元素设置为1.0f
 *       3. 必须确保rows*cols不超过MAX_MATRIX_SIZE
 */
void Matrixf_ones(Matrixf* mat, uint32_t rows, uint32_t cols) {
    if (mat == NULL || rows * cols > MAX_MATRIX_SIZE) {
        return;
    }
    
    Matrixf_init(mat, rows, cols, NULL);
    for (uint32_t i = 0; i < rows * cols; i++) {
        mat->data[i] = 1.0f;
    }
}

/**
 * @brief 创建单位矩阵（主对角线元素为1，其他元素为0）
 * 
 * @param mat 矩阵指针，将被初始化为单位矩阵
 * @param size 矩阵大小（行数=列数）
 * 
 * @note 1. 单位矩阵是一个方阵
 *       2. 首先调用Matrixf_init初始化矩阵（设为零矩阵）
 *       3. 然后将主对角线上的元素（i,i）设置为1.0f
 *       4. 必须确保size*size不超过MAX_MATRIX_SIZE
 */
void Matrixf_eye(Matrixf* mat, uint32_t size) {
    if (mat == NULL || size * size > MAX_MATRIX_SIZE) {
        return;
    }
    
    Matrixf_init(mat, size, size, NULL);
    for (uint32_t i = 0; i < size; i++) {
        mat->data[i * size + i] = 1.0f;
    }

}

/**
 * @brief 创建对角矩阵（对角线上的元素由vec提供，其他元素为0）
 * 
 * @param mat 矩阵指针，将被初始化为对角矩阵
 * @param vec 包含对角线元素的一维数组
 * @param size 矩阵大小（行数=列数）
 * 
 * @note 1. 对角矩阵是一个方阵
 *       2. 首先调用Matrixf_init初始化矩阵（设为零矩阵）
 *       3. 然后将主对角线上的元素设置为vec数组中的对应值
 *       4. 必须确保size*size不超过MAX_MATRIX_SIZE
 */
void Matrixf_diag(Matrixf* mat, const float* vec, uint32_t size) {
    if (mat == NULL || vec == NULL || size * size > MAX_MATRIX_SIZE) {
        return;
    }
    
    Matrixf_init(mat, size, size, NULL);
    for (uint32_t i = 0; i < size; i++) {
        mat->data[i * size + i] = vec[i];
    }
}

/**
 * @brief 计算矩阵的迹（主对角线元素之和）
 * 
 * @param mat 矩阵指针
 * @return 矩阵的迹
 * 
 * @note 1. 仅方阵（行数=列数）有迹，但此函数对非方阵也能正确计算（取最小维度）
 *       2. 遍历主对角线元素（i,i）并求和
 */
float Matrixf_trace(const Matrixf* mat) {
    if (mat == NULL) {
        return 0.0f;
    }
    
    float trace = 0.0f;
    uint32_t min_dim = (mat->rows < mat->cols) ? mat->rows : mat->cols;
    
    for (uint32_t i = 0; i < min_dim; i++) {
        trace += mat->data[i * mat->cols + i];
    }
    
    return trace;
}

/**
 * @brief 计算矩阵的Frobenius范数（元素平方和的平方根）
 * 
 * @param mat 矩阵指针
 * @return 矩阵的Frobenius范数
 * 
 * @note 1. Frobenius范数计算公式：||A||_F = sqrt(Σ|a_ij|2)
 *       2. 这里通过计算A^T × A 的(0,0)元素的平方根来实现
 *       3. 对结果取绝对值后开平方，确保结果非负
 */
float Matrixf_norm(const Matrixf* mat) {
    if (mat == NULL) {
        return 0.0f;
    }
    
    // 计算范数：sqrt(mat^T * mat 的第一个元素)
    Matrixf trans, product;
    Matrixf_trans(&trans, mat);
    Matrixf_mult(&product, &trans, mat);
    
    if (product.rows > 0 && product.cols > 0) {
        return sqrtf(fabsf(product.data[0]));
    }
    
    return 0.0f;
}

/**
 * @brief 比较两个矩阵是否相等
 * 
 * @param mat1 第一个矩阵
 * @param mat2 第二个矩阵
 * @return 如果两个矩阵大小相同且所有对应元素相等，则返回true；否则返回false
 * 
 * @note 1. 首先检查两个矩阵的大小是否相同
 *       2. 然后逐元素比较
 *       3. 当前实现使用精确比较，对于浮点数可能需要考虑精度误差
 */
bool Matrixf_equal(const Matrixf* mat1, const Matrixf* mat2) {
    if (mat1 == NULL || mat2 == NULL) {
        return false;
    }
    
    if (mat1->rows != mat2->rows || mat1->cols != mat2->cols) {
        return false;
    }
    
    for (uint32_t i = 0; i < mat1->rows * mat1->cols; i++) {
        if (mat1->data[i] != mat2->data[i]) {
            return false;
        }
    }
    
    return true;
}

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
 * @note 1. 首先检查指针有效性和子矩阵范围是否在源矩阵内
 *       2. 初始化结果矩阵
 *       3. 通过双重循环复制子矩阵数据
 */
void Matrixf_block(Matrixf* result, const Matrixf* src, uint32_t start_row, uint32_t start_col, uint32_t rows, uint32_t cols) {
    if (result == NULL || src == NULL) {
        return;
    }
    
    // 检查边界
    if (start_row + rows > src->rows || start_col + cols > src->cols) {
        return;
    }
    
    Matrixf_init(result, rows, cols, NULL);
    
    // 复制数据
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < cols; j++) {
            result->data[i * cols + j] = src->data[(start_row + i) * src->cols + (start_col + j)];
        }
    }
}

/**
 * @brief 提取矩阵的一行作为向量
 * 
 * @param result 结果向量（1×cols的矩阵），存储提取的行
 * @param src 源矩阵
 * @param row 要提取的行索引
 * 
 * @note 直接调用Matrixf_block函数，指定起始位置为(row, 0)，大小为(1, cols)
 */
void Matrixf_row(Matrixf* result, const Matrixf* src, uint32_t row) {
    if (result == NULL || src == NULL || row >= src->rows) {
        return;
    }
    
    Matrixf_block(result, src, row, 0, 1, src->cols);
}

/**
 * @brief 提取矩阵的一列作为向量
 * 
 * @param result 结果向量（rows×1的矩阵），存储提取的列
 * @param src 源矩阵
 * @param col 要提取的列索引
 * 
 * @note 直接调用Matrixf_block函数，指定起始位置为(0, col)，大小为(rows, 1)
 */
void Matrixf_col(Matrixf* result, const Matrixf* src, uint32_t col) {
    if (result == NULL || src == NULL || col >= src->cols) {
        return;
    }
    
    Matrixf_block(result, src, 0, col, src->rows, 1);
}

