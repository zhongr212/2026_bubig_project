/**
  ****************************(C) COPYRIGHT 2024  AIR_RM************************
  * @file       Gimbal.c/h
  * @brief      Gimbal_task,
  *             ???????????
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     4-9-2021     	JackyJuu		1. done
  *	 V3.0.0		3-23-2024       Highlight		1. done
  @verbatim
  ==============================================================================
  *  V4.0.0     12-9-2024     	Ideal 			1. hal done
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024  AIR_RM************************
  */
#ifndef CONTROL_GAIMBAL_TASK_H
#define CONTROL_GAIMBAL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "PID.h"
#define  Two_Yaw_Motor 1U 

/**
 * @brief typedef enum that contains the type of Gimbal mode.
 * @note In all modes except GIMBAL_UPPER_COMPUTER, the target values are provided by the remote controller.
 */
typedef enum{
    GIMBAL_SLEEP,               //休眠模式
    GIMBAL_REMOTE,              //遥控模式

	
}Control_Gimbal_Mode_Type_e;


#if Two_Yaw_Motor 
/**
 * @brief typedef structure that contains the information of Gimbal control
*/

typedef struct 
{
	struct{
		float Yaw_Velocity;
		float Yaw_Angle;

    float Pitch_Velocity;
    float Pitch_Angle;
	}Target;	
	struct{
    float Yaw_IMU_Angle;
		
    float Yaw_IMU_Gyro;	
    //相较于新中点
    float Pitch_Motor_Angle;
    float Yaw_Motor_Angle;
    //云台角速度-机械角速度值
    float Yaw_Motor_Gyro;	
		
    float Pitch_Motor_Gyro;	
	}Measure;

	Control_Gimbal_Mode_Type_e Type;   /*!< Type of mode */
	int16_t SendValue[3];//send current values

}Control_Gimbal_Info_Typedef;
#else
typedef struct 
{
	struct{
		float Upper_Yaw_Velocity;
		float Upper_Yaw_Angle;

    float Pitch_Velocity;
    float Pitch_Angle;
	}Target;	
	struct{
    //陀螺仪角度获取地址
    const float* Gimbal_IMU_Angle;
    //陀螺仪角速度获取地址
	  const float* Gimbal_IMU_Gyro;

    //原电机角度相较于新设中点角度的偏移量
    float Upper_Yaw_Angle_Offset;
    float Pitch_Angle_Offset;
    //云台角速度-机械角速度值
    float Upper_Yaw_Motor_Gyro;	
    float Pitch_Motor_Gyro;	
	}Measure;
	Control_Gimbal_Mode_Type_e Type;   /*!< Type of mode */
	int16_t SendValue[4];//send current values
	
}Control_Gimbal_Info_Typedef;
#endif

extern Control_Gimbal_Info_Typedef Control_Gimbal_Info;
extern PID_Info_TypeDef UpperYaw_IMU_Angle_PID;
#endif
