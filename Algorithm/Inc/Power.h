#ifndef POWER_H
#define POWER_H

#include "Math.h"                  // 数学函数
#include "PID.h"                   // PID控制器
#include "RLS.h"                   // 递推最小二乘法参数估计器
#include "Matrix.h"

#define capFullBuffSet              230.0f // 电容满缓冲阈值 (J)
#define capBaseBuffSet              30.0f  // 电容基础缓冲阈值 (J)

// 功率分配参数
#define error_powerDistribution_set 20.0f  // 误差功率分配系数
#define prop_powerDistribution_set  15.0f  // 比例功率分配系数

// 电容相关功率限制
#define MAX_CAP_POWER_OUT                         300.0f  // 电容最大输出功率 (W)
#define CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD 43.0f   // 电容离线且能量耗尽时的功率阈值 (W)
#define CAP_OFFLINE_ENERGY_TARGET_POWER           37.0f   // 电容离线且能量耗尽时的目标功率 (W)

// 功率缓冲与保护系数
#define MAX_POEWR_REFEREE_BUFF                    60.0f  // 裁判系统最大功率缓冲 (J)
#define REFEREE_GG_COE                            0.95f  // 仅裁判系统时的缓冲系数
#define CAP_REFEREE_BOTH_GG_COE                   0.85f  // 裁判系统和电容同时使用时的缓冲系数
#define POWER_PD_KP                               50.0f  // 功率PD控制器比例系数

// 机器人等级最大功率限制表
#define maxLevel 10U  // 最大等级数

// 英雄机器人不同等级的底盘功率限制 (W)
static const uint8_t HeroChassisPowerLimit_HP_FIRST[maxLevel] = {55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U};

// 步兵机器人不同等级的底盘功率限制 (W)
static const uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel] = {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};

// 哨兵机器人底盘功率限制 (W)
#define SentryChassisPowerLimit 100U

// 机器人类型枚举
enum Division {
    INFANTRY = 0,  // 步兵机器人
    HERO,          // 英雄机器人
    SENTRY         // 哨兵机器人
};

// RLS启用状态枚举
enum RLSEnabled {
    RLS_DISABLE = 0,  // 禁用RLS自适应参数估计
    RLS_ENABLE = 1    // 启用RLS自适应参数估计
};

// 错误标志枚举（使用位掩码方式）
enum ErrorFlags {
    MOTOR_DISCONNECT = 1U,      // 电机断开连接 (0x01)
    REFEREE_DISCONNECT = 2U,    // 裁判系统断开连接 (0x02)
    CAP_DISCONNECT = 4U         // 电容断开连接 (0x04)
    // 可以通过位或运算组合多个错误
};

/**
 * @brief 功率管理器结构体
 * 
 * 存储功率控制器的核心配置和状态信息，是功率控制的主要数据结构
 */
struct Manager {
    enum RLSEnabled rlsEnabled;  // RLS自适应参数模式启用状态
    uint8_t error;               // 错误标志 (使用ErrorFlags枚举值的位掩码)
    enum Division division;      // 机器人类型
    
    // 功率缓冲相关参数
    float powerBuff;             // 当前功率缓冲值 (J) 4
    float fullBuffSet;           // 满缓冲设置值 (J)
    float baseBuffSet;           // 基础缓冲设置值 (J)
    float fullMaxPower;          // 满缓冲时的最大允许功率 (W)
    float baseMaxPower;          // 基础缓冲时的最大允许功率 (W)
    
    float powerUpperLimit;       // 当前功率上限 (W)
    float refereeMaxPower;       // 裁判系统最大允许功率 (W)
    
    float userConfiguredMaxPower;// 用户配置的最大功率 (W)
    float (*callback)(void);     // 功率获取回调函数
    
    float measuredPower;         // 当前测量功率 (W)
    float estimatedPower;        // 估算功率 (W)
    float estimatedCapEnergy;    // 估算电容能量 (J)
    
    // 功率损耗模型参数
    float torqueConst;           // 电机转矩常数
    float k1;                    // 频率损耗参数 (与速度相关的损耗系数)
    float k2;                    // 电流损耗平方参数 (与电流平方相关的损耗系数)
    float k3;                    // 恒定功率损耗 (静态损耗)
    
    TickType_t lastUpdateTick;   // 上次更新时间戳
		RLS rls;              // RLS自适应参数估计器实例
};

/**
 * @brief 功率对象结构体
 * 
 * 存储单个电机的功率控制相关数据
 */
 struct PowerObj{
    float pidOutput;     // 电机PID控制器的输出 (转矩电流命令)，范围[-maxOutput, maxOutput]
    float curAv;         // 电机当前测量角速度 (rad/s)
    float setAv;         // 电机目标角速度 (rad/s)
    float pidMaxOutput;  // PID控制器的最大输出限制
};

/**
 * @brief 功率状态结构体
 * 
 * 存储底盘整体功率状态信息，可用于监控和调试
 */
struct PowerStatus {
    float userConfiguredMaxPower;    // 用户配置的最大功率 (W)
    float maxPowerLimited;           // 限制后的实际最大功率 (W)
    float sumPowerCmd_before_clamp;  // 功率限制前的总功率命令 (W)
    float effectivePower;            // 有效功率，实际用于驱动的功率 (W)
    float powerLoss;                 // 功率损耗，系统损失的功率 (W)
    float efficiency;                // 效率，有效功率与总功率的比值 (0-1)
    uint8_t estimatedCapEnergy;      // 估算的电容能量 (J)
    enum ErrorFlags error;           // 错误标志
};

/**
 * @brief 获取底盘的当前功率状态
 * 
 * @return 功率状态结构体指针，包含当前功率、效率、错误标志等信息
 * 
 * @note 可以使用此函数获取功率系统的运行状态，用于监控和调试
 */
//const volatile struct PowerStatus* Power_getPowerStatus(void);

/**
 * @brief 初始化功率管理器结构体
 * 
 * @param manager 管理器结构体指针，将被初始化
 * @param motors 电机数据结构体，包含所有电机指针
 * @param division 机器人类型枚举值（INFANTRY, HERO, SENTRY）
 * @param rlsEnabled RLS参数自适应启用状态
 * @param k1 频率损耗参数初始值
 * @param k2 电流损耗平方参数初始值
 * @param k3 恒定功率损耗初始值
 * @param lambda RLS更新遗忘因子，范围(0,1]，接近1时遗忘速度慢
 * 
 * @note 此函数用于配置功率管理器的初始参数，是使用功率控制器前的必要步骤
 */
void Manager_init(struct Manager* manager,
                  enum Division division,
                  enum RLSEnabled rlsEnabled,
                  float k1,
                  float k2,
                  float k3);

									
float* Power_getControlledOutput(struct PowerObj* objs);
void RLS_task(struct Manager* manager, const float* sampleVector, float actualOutput);
/**
 * @brief 初始化功率对象数组
 */
//void PowerObj_init(struct PowerObj* objs);

/**
 * @brief 浮点数比较函数
 */
int floatEqual(float a, float b);
									
float Utils_Math_clamp(float value, float min, float max);
void energy_control(struct Manager* manager,float Kp,float Kd);
									
extern struct Manager manager; 					

									
#endif	



