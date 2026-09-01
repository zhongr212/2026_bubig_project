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

#ifndef RLS_H
#define RLS_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Matrix.h"

// 定义最大RLS维度
// 注意：增加此值会增加内存占用，根据实际应用需求调整
#define MAX_RLS_DIMENSION 10

/**
 * @brief 递归最小二乘（RLS）结构体定义
 * 
 * 此结构体封装了RLS算法所需的所有状态和参数
 * RLS算法用于在线估计系统参数，通过递归方式更新估计结果
 */
typedef struct {
    uint32_t dimension;          // RLS空间维度，即系统参数的数量
    float lambda;                // 遗忘因子（0 < lambda ≤ 1），控制历史数据的权重
                                // lambda接近1时，历史数据权重较大；lambda较小时，更重视新数据
    float delta;                 // 转移矩阵的初始值，影响算法初始收敛性能
    TickType_t lastUpdate;       // 上次更新的tick，用于时间间隔计算
    uint32_t updateCnt;          // 总更新次数，记录算法迭代次数
    
    // RLS相关矩阵
    Matrixf transMatrix;         // 转移矩阵（P矩阵），维度为dimension×dimension
                                // 代表参数估计的协方差矩阵，反映参数估计的不确定性
    Matrixf gainVector;          // 参数更新的增益向量（K向量），维度为dimension×1
                                // 决定了每次更新时参数调整的幅度和方向
    Matrixf paramsVector;        // 参数向量（θ向量），维度为dimension×1
                                // 存储当前估计的系统参数
    Matrixf defaultParamsVector; // 默认参数向量，用于重置时恢复初始参数
    float output;                // 估计/滤波输出，即系统预测值 y = φ^T * θ
} RLS;

/**
 * @brief 初始化RLS模块
 * 
 * @param rls RLS结构体指针
 * @param dimension 维度，表示系统参数的数量
 * @param delta 转移矩阵的初始非奇异值，通常设为较大的正数
 * @param lambda 遗忘因子（0 < lambda ≤ 1）
 * 
 * @note 1. 维度必须小于等于MAX_RLS_DIMENSION
 *       2. 遗忘因子lambda接近1时算法更稳定，但对时变系统跟踪能力较弱
 *       3. delta通常设为一个较大的正数（如1000），确保初始P矩阵正定
 */
void RLS_init(RLS* rls, uint32_t dimension, float delta, float lambda);

/**
 * @brief 使用初始参数向量初始化RLS模块
 * 
 * @param rls RLS结构体指针
 * @param dimension 维度，表示系统参数的数量
 * @param delta 转移矩阵的初始非奇异值
 * @param lambda 遗忘因子（0 < lambda ≤ 1）
 * @param initParam 初始参数向量，提供算法初始估计值
 * 
 * @note 1. 当初始参数已知或有较好的估计时，使用此函数可加速收敛
 *       2. 初始参数向量的大小必须为dimension
 */
//void RLS_initWithParam(RLS* rls, uint32_t dimension, float delta, float lambda, const float* initParam);

/**
 * @brief 重置RLS模块
 * 
 * @param rls RLS结构体指针
 * 
 * @note 1. 重置后，转移矩阵恢复为初始状态（delta*I）
 *       2. 参数向量恢复为默认值（通过RLS_init初始化时为零，通过RLS_initWithParam初始化时为指定值）
 *       3. 更新计数和时间戳也会被重置
 */
//void RLS_reset(RLS* rls);

/**
 * @brief 执行一次RLS更新
 * 
 * 这是RLS算法的核心函数，按照递归最小二乘算法更新参数估计
 * 算法步骤：
 * 1. 计算增益向量：K = P * φ / (λ + φ^T * P * φ)
 * 2. 更新参数向量：θ = θ + K * (y - φ^T * θ)
 * 3. 更新转移矩阵：P = (P - K * φ^T * P) / λ
 * 
 * @param rls RLS结构体指针
 * @param sampleVector 新的样本输入（φ向量），维度为dimension x 1
 * @param actualOutput 实际反馈输出（y值）
 * @return 更新后的参数向量
 * 
 * @note 1. sampleVector必须包含dimension个元素
 *       2. 每次调用此函数都会更新RLS的内部状态
 */
const Matrixf* RLS_update(RLS* rls, const float* sampleVector, float actualOutput);

/**
 * @brief 设置默认回归参数
 * 
 * @param rls RLS结构体指针
 * @param updatedParams 更新的参数向量
 * 
 * @note 1. 此函数同时更新当前参数向量和默认参数向量
 *       2. updatedParams必须包含dimension个元素
 *       3. 调用此函数不会重置转移矩阵P
 */
void RLS_setParamVector(RLS* rls, const float* updatedParams);

/**
 * @brief 获取参数向量
 * 
 * @param rls RLS结构体指针
 * @return 当前估计的参数向量（θ向量），维度为dimension x 1
 * 
 * @note 返回的是指针，调用者不应修改其内容
 */
//const Matrixf* RLS_getParamsVector(const RLS* rls);

/**
 * @brief 获取估计输出
 * 
 * @param rls RLS结构体指针
 * @return 上一次更新时的估计输出值 y = φ^T * θ
 * 
 * @note 此值是在RLS_update函数中计算的
 */
//float RLS_getOutput(const RLS* rls);

/**
 * @brief 验证RLS参数的有效性
 * 
 * @param rls RLS结构体指针
 * @return 如果参数有效（lambda在合理范围内且delta为正数），返回true；否则返回false
 * 
 * @note 有效的lambda值应满足 0 < lambda ≤ 1
 *       有效的delta值应满足 delta > 0
 */
bool RLS_validate(const RLS* rls);

#endif /* RLS_H */

//#ifndef RLS_H
//#define RLS_H


//#include "Config.h"
//#include "stdint.h"
//#include "arm_math.h"

//#define Matrix             arm_matrix_instance_f32
//#define Matrix_64          arm_matrix_instance_f64
//#define Matrix_Init        arm_mat_init_f32
//#define Matrix_Add         arm_mat_add_f32
//#define Matrix_Subtract    arm_mat_sub_f32
//#define Matrix_Multiply    arm_mat_mult_f32
//#define Matrix_Transpose   arm_mat_trans_f32
//#define Matrix_Inverse     arm_mat_inverse_f32
//#define Matrix_Inverse_64  arm_mat_inverse_f64


//typedef struct
//{

//  uint8_t sizeof_float;
//	
//	uint8_t X_Size;
//	uint8_t Y_Size;
//	uint8_t P_Size;
//	float Lamda; //forgetting factor

// struct 
//  {
//    Matrix X; 
//    Matrix XT; 
//    Matrix Lamda; 		
//    Matrix E;                 
//    Matrix Z;
//    Matrix K;  		
//    Matrix W;         		
//    Matrix P;
//    Matrix Y;
//    Matrix U;		
//    Matrix K_Numerator;                 
//    Matrix K_Denominator;    
//    Matrix Cache_Matrix[2];   
//    Matrix Cache_Vector[2];   
//		Matrix Output;  
//  }Mat;
//	
//	 arm_status MatStatus;
//	
//	 struct 
//  {
//    float *X;
//    float	*XT;
//    float *Lamda;	
//    float *E;                 
//    float *Z;
//    float *K;		
//    float *W; 
//		float *Y;		
//    float *U;                 
//    float *P;                 
//    float *K_Numerator;                 
//    float *K_Denominator;    
//    float *Cache_Matrix[2];   
//    float *Cache_Vector[2];  
//    float *Output;
// 		
//  }Data;

//	
//	
//	
//}Recursive_Least_Squares_Info_TypeDef;


//extern Recursive_Least_Squares_Info_TypeDef Power_Control_RLS_Info;

//extern void RLS_Update(Recursive_Least_Squares_Info_TypeDef *RLS);
//extern void RLS_Init(Recursive_Least_Squares_Info_TypeDef *RLS,uint8_t X_Size,uint8_t P_Size,uint8_t Y_Size);




//#endif
