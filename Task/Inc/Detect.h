#ifndef __DETECT_H
#define __DETECT_H


#include "stdint.h"
#include "stdbool.h" 
#include "stm32h7xx.h"                  // Device header
#include "string.h"

typedef struct
{     

  float pitch_angle_get_temp;
	float yaw_angle_get_temp;
	
	int gimbal_state;
	uint8_t receive_finish_flag;
  
//	bool upper_lost;   /*!< lost flag */
//	uint32_t online_cnt;   /*!< online count */
} Upper_Info_Typedef;

extern void MiniPC_Transmit_Info(float yaw_data,float pitch_data);
extern Upper_Info_Typedef HMI_Contrl_update;

#endif