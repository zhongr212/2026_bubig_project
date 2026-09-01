/**
 ******************************************************************************
 * @file    RLS.c/h
 * @brief   递归最小二乘（RLS）算法实现
 *          提供高效的自适应参数估计和系统辨识功能，适用于实时控制系统
 *          RLS算法比传统的最小二乘法有更快的收敛速度和更好的跟踪能力
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "RLS.h"
#include <stdlib.h>


float k_pram[2];
/**
 * @brief 验证RLS参数的有效性
 * 
 * 检查RLS结构体中的关键参数（lambda和delta）是否在有效范围内
 * 
 * @param rls RLS结构体指针
 * @return 如果参数有效，返回true；否则返回false
 * 
 * @note 有效的lambda值应满足 0 < lambda ≤ 1
 *       有效的delta值应满足 delta > 0
 */
bool RLS_validate(const RLS* rls) {
    if (rls == NULL) {
        return false;
    }
    
    // 验证lambda是否在0到1之间
    if (rls->lambda < 0.0f || rls->lambda > 1.0f) {
        return false;
    }
    
    // 验证delta是否大于0
    if (rls->delta <= 0.0f) {
        return false;
    }
    
    return true;
}

/**
 * @brief 初始化RLS模块
 * 
 * 设置RLS算法的基本参数，并初始化所有相关矩阵和向量
 * 
 * @param rls RLS结构体指针
 * @param dimension 维度，表示系统参数的数量
 * @param delta 转移矩阵的初始非奇异值，通常设为较大的正数
 * @param lambda 遗忘因子（0 < lambda ≤ 1）
 * 
 * @note 1. 首先验证输入参数的有效性
 *       2. 初始化转移矩阵为 delta * 单位矩阵
 *       3. 初始化其他向量为零向量
 *       4. 重置计数器和输出值
 */
void RLS_init(RLS* rls, uint32_t dimension, float delta, float lambda) {
    if (rls == NULL || dimension > MAX_RLS_DIMENSION) {
        return;
    }
    
    // 初始化基本参数
    rls->dimension = dimension;
    rls->lambda = lambda;
    rls->delta = delta;
    rls->lastUpdate = 0;
    rls->updateCnt = 0;
    rls->output = 0.0f;
    
    // 初始化矩阵
    Matrixf_eye(&rls->transMatrix, dimension);
		Matrixf temp4;
		Matrixf_eye(&temp4,dimension);
    Matrixf_scale(&rls->transMatrix, &temp4, delta);  // P = delta * I
    
    Matrixf_zeros(&rls->gainVector, dimension, 1);
    Matrixf_zeros(&rls->paramsVector, dimension, 1);
    Matrixf_zeros(&rls->defaultParamsVector, dimension, 1);
    
    // 验证参数
    RLS_validate(rls);
}

/**
 * @brief 使用初始参数向量初始化RLS模块
 * 
 * 在基本初始化的基础上，设置自定义的初始参数向量
 * 
 * @param rls RLS结构体指针
 * @param dimension 维度，表示系统参数的数量
 * @param delta 转移矩阵的初始非奇异值
 * @param lambda 遗忘因子（0 < lambda ≤ 1）
 * @param initParam 初始参数向量，提供算法初始估计值
 * 
 * @note 1. 首先调用RLS_init进行基本初始化
 *       2. 然后设置初始参数向量和默认参数向量
 *       3. 当初始参数已知时，使用此函数可加速算法收敛
 */
//void RLS_initWithParam(RLS* rls, uint32_t dimension, float delta, float lambda, const float* initParam) {
//    // 先进行基本初始化
//    RLS_init(rls, dimension, delta, lambda);
//    
//    // 设置初始参数向量
//    if (initParam != NULL) {
//        for (uint32_t i = 0; i < dimension; i++) {
//            Matrixf_set(&rls->paramsVector, i, 0, initParam[i]);
//            Matrixf_set(&rls->defaultParamsVector, i, 0, initParam[i]);
//        }
//    }
//}

/**
 * @brief 重置RLS模块
 * 
 * 将RLS算法的状态恢复到初始化时的状态
 * 
 * @param rls RLS结构体指针
 * 
 * @note 1. 转移矩阵恢复为 delta * 单位矩阵
 *       2. 增益向量重置为零向量
 *       3. 参数向量恢复为默认值（初始化时设置的值）
 *       4. 重置计数器和输出值
 */
//void RLS_reset(RLS* rls) {
//    if (rls == NULL) {
//        return;
//    }
//    
//    // 重置转移矩阵 P = delta * I
//    Matrixf_eye(&rls->transMatrix, rls->dimension);
//		Matrixf temp4;
//		Matrixf_eye(&temp4,rls->dimension);
//    Matrixf_scale(&rls->transMatrix, &temp4, rls->delta);  // P = delta * I
//    
//    // 重置其他向量
//    Matrixf_zeros(&rls->gainVector, rls->dimension, 1);
//    
//    // 重置为默认参数向量
//    Matrixf_copy(&rls->paramsVector, &rls->defaultParamsVector);
//    
//    // 重置计数器和输出
//    rls->updateCnt = 0;
//    rls->output = 0.0f;
//}

/**
 * @brief 执行一次RLS更新
 * 
 * 这是RLS算法的核心实现，按照递归最小二乘算法的标准步骤更新参数估计
 * 算法核心公式：
 * 1. K(k) = P(k-1)φ(k) / (λ + φ^T(k)P(k-1)φ(k))
 * 2. θ(k) = θ(k-1) + K(k)(y(k) - φ^T(k)θ(k-1))
 * 3. P(k) = (P(k-1) - K(k)φ^T(k)P(k-1))/λ
 * 
 * @param rls RLS结构体指针
 * @param sampleVector 新的样本输入（φ向量），维度为dimension x 1
 * @param actualOutput 实际反馈输出（y值）
 * @return 更新后的参数向量
 * 
 * @note 1. 首先验证输入参数的有效性
 *       2. 创建样本向量及其转置矩阵
 *       3. 计算并更新增益向量
 *       4. 计算预测误差
 *       5. 更新参数向量
 *       6. 更新转移矩阵
 *       7. 更新计数器和时间戳
 */
const Matrixf* RLS_update(RLS* rls, const float* sampleVector, float actualOutput) {
    if (rls == NULL || sampleVector == NULL) {
        return NULL;
    }
    
    // 创建样本向量矩阵
    Matrixf sampleVecMat;
    Matrixf_init(&sampleVecMat, rls->dimension, 1, sampleVector);
    
    // 创建样本向量的转置
    Matrixf sampleVecTrans;
    Matrixf_trans(&sampleVecTrans, &sampleVecMat);
    
    // 计算中间变量：sampleVecTrans * transMatrix * sampleVector
    Matrixf temp1, temp2;
    Matrixf_mult(&temp1, &sampleVecTrans, &rls->transMatrix);
    Matrixf_mult(&temp2, &temp1, &sampleVecMat);
    
    // 获取标量值
    float tempScalar = Matrixf_get(&temp2, 0, 0);
    
    // 计算增益向量：K = (P * sampleVector) / (lambda + sampleVector^T * P * sampleVector)
    Matrixf_mult(&temp1, &rls->transMatrix, &sampleVecMat);  // P * sampleVector
    float denominator = rls->lambda + tempScalar;
    
    if (denominator != 0.0f) {
        Matrixf_scale(&rls->gainVector, &temp1, 1.0f / denominator);
    }
    
    // 计算误差：e = actualOutput - sampleVector^T * paramsVector
    Matrixf_mult(&temp2, &sampleVecTrans, &rls->paramsVector);
    float error = actualOutput - Matrixf_get(&temp2, 0, 0);
    
    // 更新参数向量：theta = theta + K * e
    Matrixf temp3;
    Matrixf_scale(&temp3, &rls->gainVector, error);
    Matrixf_add(&rls->paramsVector, &rls->paramsVector, &temp3);
    
    // 更新转移矩阵：P = (P - K * sampleVector^T * P) / lambda
    Matrixf_mult(&temp1, &rls->gainVector, &sampleVecTrans);  // K * sampleVector^T
    Matrixf_mult(&temp2, &temp1, &rls->transMatrix);         // K * sampleVector^T * P
    Matrixf_sub(&temp3, &rls->transMatrix, &temp2);          // P - K * sampleVector^T * P
    Matrixf_scale(&rls->transMatrix, &temp3, 1.0f / rls->lambda);  // 除以lambda
    
    // 更新计数器和时间戳
    rls->updateCnt++;
    rls->lastUpdate = xTaskGetTickCount();
    
    // 更新输出
    rls->output = actualOutput;
    
    return &rls->paramsVector;
}

/**
 * @brief 设置默认回归参数
 * 
 * 更新当前参数向量和默认参数向量
 * 
 * @param rls RLS结构体指针
 * @param updatedParams 更新的参数向量
 * 
 * @note 1. 同时更新当前参数和默认参数，以便在重置时使用
 *       2. 此操作不会重置转移矩阵P，仅更新参数向量
 */
void RLS_setParamVector(RLS* rls, const float* updatedParams) {
    if (rls == NULL || updatedParams == NULL) {
        return;
    }
    
    // 设置参数向量
    for (uint32_t i = 0; i < rls->dimension; i++) {
        Matrixf_set(&rls->paramsVector, i, 0, updatedParams[i]);
        Matrixf_set(&rls->defaultParamsVector, i, 0, updatedParams[i]);
    }
}

/**
 * @brief 获取参数向量
 * 
 * 返回当前估计的参数向量
 * 
 * @param rls RLS结构体指针
 * @return 当前估计的参数向量（θ向量），如果rls为NULL则返回NULL
 * 
 * @note 返回的是指针，调用者不应修改其内容
 */
//const Matrixf* RLS_getParamsVector(const RLS* rls) {
//    if (rls == NULL) {
//        return NULL;
//    }
//    
//    return &rls->paramsVector;
//}

/**
 * @brief 获取估计输出
 * 
 * 返回上一次更新时的实际输出值
 * 
 * @param rls RLS结构体指针
 * @return 上一次更新时的实际输出值，如果rls为NULL则返回0.0f
 * 
 * @note 此值在每次RLS_update调用中被更新为传入的actualOutput
 */
//float RLS_getOutput(const RLS* rls) {
//    if (rls == NULL) {
//        return 0.0f;
//    }
//    
//    return rls->output;
//}
