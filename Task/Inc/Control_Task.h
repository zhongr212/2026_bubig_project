/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"


/**
 * @brief typedef structure that contains the information of chassis control
*/

typedef enum
{
	sleep,
	chassis_mode,
	chassis_follow_gimbal,
	chassis_spin
}Control_Mode_Typedef;

typedef enum
{
	on,
	off
}v_control_Typedef;

typedef struct 
{
	    /*  X:move forward (m/s),Y:Move left (m/s),Theta:ratate (rpm)*/
	struct{
		float Wheel_Velocity[4];
    float velX;
    float velY;
    float velTheta;
	}Target;	
	struct{
  	float	Wheel_Velocity[4];

    float velTheta;

	}Measure;
  struct{
    uint16_t switch_count;       // 切换计数（记录从中档到上档再回到中档的次数）
    uint16_t last_mid_state;     // 上次中档状态（用于检测状态变化）
  } SentryController;
	Control_Mode_Typedef Type;   /*!< Type of mode */
  float* Chassis_Follow_Gimbal_Angle_TM;
	int16_t SendValue[4];//send current values
	
}Chassis_Info_Typedef;

extern Chassis_Info_Typedef Chassis_Info;

#endif //CONTROL_TASK_H
