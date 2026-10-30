#ifndef POWER_CTRL
#define POWER_CTRL
#define POWERCTRL_TYPE_NUM 5
#include "RLS.h"
#include "motor.h"
#include "Referee_System.h"
#include "math.h"

typedef enum
{
    MACUNUM,
    STEER,
    CHASSIS_TYPE_NUM,
}CHASSIS_TYPE_ENUM;

typedef struct
{
float K1;
float K2;
float K3;
float Err_Lower;
float Err_Upper;
} PowerCtrl_Parameter_Typedef;

typedef struct
{

    // 经验公式系数

    float A;
    float B;
    float C;
    // 最小二乘法系数
    float Delta;
    float Sqrt;
    // 一元二次方程中间变量

    float K;
    float Power_Max;      // 最大能量
    float Power_Allin;    // 期望输出总功率
    float Power_Limit[4]; // 输出功率限制
    float Menbership[4];  // 隶属度
    float Torque[4];      // 输出力矩


    float Err[4];    // 期望输出电流于电机实际电流
    struct
    {
        float Torque2_Sum;
        float Omiga2_Sum;
        float Power_Sum;
        float Err_Sum;
    } Sum; // 一些数据求和
    struct
    {
        float Power_In[4];
        float Torque[4];
        float Omiga[4];
        float Torque_2[4];
        float Omiga_2[4];
    } Target; // 模型输入的参数
    float Output[4];
    PowerCtrl_Parameter_Typedef Param;

} PowerCtrl_Typedef;
extern PowerCtrl_Typedef PowerCtrl_Info;
extern void PowerCtrl_Init(PowerCtrl_Typedef *PowerCtrl_Info, float Lamda, float P, float PowerCtrl_Param[POWERCTRL_TYPE_NUM]);
extern void PowerCtrl(PowerCtrl_Typedef *PowerCtrl_Info, PID_Info_TypeDef *Pid, DJI_Motor_Info_Typedef *Chassis_Motor);
#endif