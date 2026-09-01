/**
 * @file    Power.c
 * @brief   功率管理系统实现文件
 *          提供底盘电机的功率估算、限制和能量优化功能
 *          支持超级电容能量管理、裁判系统功率限制响应和RLS参数自学习
 */
#include "Power.h"
#include <math.h>
//#include "Motor.h"

struct Manager manager;              // 功率管理器结构体
struct PowerStatus powerStatus;     // 功率状态结构体
struct PowerObj powerObjs[4]; 
float sumPowerCmd;
/**
 * @brief 初始化管理器结构体
 */
void Manager_init(struct Manager* manager,
                  enum Division division,
                  enum RLSEnabled rlsEnabled,
									float k1,
                  float k2,
                  float k3)//初始化，RLS k1,k2,k3传值
{
   
    
    // 初始化成员变量
    manager->rlsEnabled = rlsEnabled;
    manager->error = 0UL;
    manager->division = division;
    manager->powerBuff = 0.0f;
    manager->fullBuffSet = 0.0f;
    manager->baseBuffSet = 0.0f;
    manager->fullMaxPower = 0.0f;
    manager->baseMaxPower = 0.0f;
    manager->powerUpperLimit = 0.0f;
    manager->refereeMaxPower = 0.0f;
    manager->userConfiguredMaxPower = 0.0f;
    manager->callback = NULL;
    manager->torqueConst = 0.0f;
    
    manager->lastUpdateTick = 0;
    
    // 初始化RLS参数
    RLS_init(&manager->rls,2,0.00000001, 0.99999f);
    float initParams[2] = {k1, k2};
    manager->k3 = k3;
		Matrixf params;
    Matrixf_init(&params, 2, 1, 0);
    for (int i = 0; i < 2; i++) {
        params.data[i] = initParams[i];
    }
    RLS_setParamVector(&manager->rls, params.data);
}


/**
 * @brief 获取功率状态
 */
//const volatile struct PowerStatus* Power_getPowerStatus(void)
//{
//    return &powerStatus;
//}

/**
 * @brief 获取经过功率控制的电机输出值
 * 
 * 功率管理系统的核心函数，根据功率限制和电机需求进行智能功率分配
 * 实现原理：
 * 1. 计算每个电机的估计功率需求 P = τΩ + k1|Ω| + k2τ2 + k3/4
 * 2. 当总功率需求超过限制时，基于误差和比例进行功率分配
 * 3. 解二次方程计算新的转矩电流，确保功率在限制范围内
 * 
 * @param objs 4个电机的功率对象数组
 * @return 调整后的电机输出值数组
 * 
 * @note 1. 功率估算基于电机模型和经验系数k1、k2、k3
 *       2. 功率分配同时考虑速度误差和功率需求比例
 *       3. 电机断开连接时的处理逻辑确保系统稳定性
 */
float* Power_getControlledOutput(struct PowerObj* objs)
{
    static float newTorqueCurrent[4];
    
    // 计算电机的转矩电流比率 (Nm/Output)
    const float k0 = 0.1562*0.001220703125;

    float sumCmdPower = 0.0f;
    float cmdPower[4];
    float sumError = 0.0f;
    float error[4];
    
    // 获取最大允许功率
    float maxPower = Utils_Math_clamp(manager.userConfiguredMaxPower,manager.baseMaxPower,manager.fullMaxPower);
    float allocatablePower = maxPower;
    float sumPowerRequired = 0.0f;

    // 计算每个电机的功率需求
    for (int i = 0; i < 4; i++)
    {
     

            // 计算命令功率: P = τΩ + k1|Ω| + k2τ^2 + k3/4
            cmdPower[i] = objs[i].pidOutput * k0 * objs[i].curAv + fabs(objs[i].curAv) * manager.k1 + 
                         objs[i].pidOutput * k0 * objs[i].pidOutput * k0 * manager.k2 + manager.k3 / 4.0f;
            sumCmdPower += cmdPower[i];
            error[i] = fabs(objs[i].setAv - objs[i].curAv);
            
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
            {
                allocatablePower += -cmdPower[i];
            }
            else
            {
                sumError += error[i];
                sumPowerRequired += cmdPower[i];
            }

    }

    // 更新功率状态
    powerStatus.maxPowerLimited = maxPower;
		sumPowerCmd = sumCmdPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    // 如果总功率需求超过限制，则进行功率分配
    if (sumCmdPower > maxPower)
    {
        float errorConfidence;
        if (sumError > error_powerDistribution_set)
        {
            errorConfidence = 1.0f;
        }
        else if (sumError > prop_powerDistribution_set)
        {
            errorConfidence = Utils_Math_clamp((sumError - prop_powerDistribution_set) / 
                                             (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
        }
        else
        {
            errorConfidence = 0.0f;
        }
        
        for (int i = 0; i < 4; i++)
        {
           
                if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
                {
                    newTorqueCurrent[i] = objs[i].pidOutput;
                    continue;
                }
                
                // 计算功率权重（基于误差和比例）
                float powerWeight_Error = fabs(objs[i].setAv - objs[i].curAv) / sumError;
                float powerWeight_Prop = cmdPower[i] / sumPowerRequired;
                float powerWeight = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
                
                // 解二次方程计算新的转矩电流
                float delta = objs[i].curAv * objs[i].curAv - 4.0f * manager.k2 * 
                             (manager.k1 * fabs(objs[i].curAv) + manager.k3 / 4.0f - powerWeight * allocatablePower);
                
                if (floatEqual(delta, 0.0f))  // 重根
                {
                    newTorqueCurrent[i] = -objs[i].curAv / (2.0f * manager.k2) / k0;
                }
                else if (delta > 0.0f)  // 不同实根
                {
                    newTorqueCurrent[i] = (objs[i].pidOutput > 0.0f) ? 
                                         (-objs[i].curAv + sqrtf(delta)) / (2.0f * manager.k2) / k0:
                                         (-objs[i].curAv - sqrtf(delta)) / (2.0f * manager.k2) / k0;
                }
                else  // 复根
                {
                    newTorqueCurrent[i] = -objs[i].curAv / (2.0f * manager.k2) / k0;
                }
                
                // 限制输出在PID最大输出范围内
                newTorqueCurrent[i] = Utils_Math_clamp(newTorqueCurrent[i], -objs[i].pidMaxOutput, objs[i].pidMaxOutput);
           
        }
    }
    else
    {
        // 功率充足，直接使用PID输出
        for (int i = 0; i < 4; i++)
        {
         
            newTorqueCurrent[i] = objs[i].pidOutput;
        }
    }

    return newTorqueCurrent;
}

int floatEqual(float a, float b)
{
    return fabs(a - b) < 1e-6f;
}

/**
 * @brief 初始化功率对象数组
 */
//void PowerObj_init(struct PowerObj* objs)
//{
//    for (int i = 0; i < 4; i++)
//    {
//        objs[i].pidOutput = 0.0f;
//        objs[i].curAv = 0.0f;
//        objs[i].setAv = 0.0f;
//        objs[i].pidMaxOutput = 0.0f;  // 需要根据实际情况设置
//    }
//}

float Utils_Math_clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void RLS_task(struct Manager* manager, const float* sampleVector, float actualOutput)
{
	  const Matrixf* k;
    
    // 修复函数调用 - 移除类型声明，直接传递参数
    k = RLS_update(&manager->rls, sampleVector, actualOutput);
    
    manager->k1 = k->data[0];
    manager->k2 = k->data[1]; 
}

void energy_control(struct Manager* manager,float Kp,float Kd)//实现能量环控制
{
	float e_last_max,e_now_max,e_last_min,e_now_min;
	e_last_max = 0;
	e_last_max = e_now_max;
	e_now_max = sqrtf(manager->baseBuffSet) - sqrtf(manager->powerBuff);
	manager->baseMaxPower = fmax(manager->refereeMaxPower - Kp*e_now_max -Kd*(e_now_max - e_last_max),35.0f);
	
	e_last_min = 0;
	e_last_min = e_now_min;
	e_now_min = sqrtf(manager->fullBuffSet) - sqrtf(manager->powerBuff);
	manager->fullMaxPower = fmax(manager->refereeMaxPower - Kp*e_now_min -Kd*(e_now_min - e_last_min),35.0f);
}
