/**
  ****************************(C) COPYRIGHT 2024  AIR_RM************************
  * @file       chassis.c/h
  * @brief      chassis_task,
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
#ifndef Magazine_TASK_H
#define Magazine_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

typedef enum{
		SHOOT_ENABLE,
    SHOOT_SLEEP,
    SHOOT_SINGLE,
		SHOOT_SINGLE_WAIT,
		SHOOT_FIRE,
}Control_Magazine_Mode_Type_e;

/**
 * @brief typedef structure that contains the information of chassis control
*/



typedef struct 
{
	
	struct{
		
		float Magazine_Velocity;
		float	Magazine_Angle;
		float Shoot0_Velocity;
		float Shoot1_Velocity;
	
	}Target;	
 
	struct{
		
	 float	Magazine_Velocity;
	 float	Magazine_Angle;
		float Shoot0_Velocity;
		float Shoot1_Velocity;
	
	}Measure;
	
	Control_Magazine_Mode_Type_e Type;   /*!< Type of mode */
  int16_t fireRate;//每秒出弹个数
  int16_t fireSpeed;//射速
	int16_t SendValue[4];
	
}Control_Magazine_Info_Typedef;


extern Control_Magazine_Info_Typedef Control_Magazine_Info;
extern int cross_count;
#endif
