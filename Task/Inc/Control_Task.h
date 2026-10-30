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
#include "PowerContral.h"

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)



#define KEY_MAX_SPEED_MAX             (uint16_t)3
#define KEY_SHIFT_SPEED_MAX_MAX       (uint16_t)6
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
//		char key[16];//0 ~16  :W S A D shift Ctrl Q E R F G Z X C V B 
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
