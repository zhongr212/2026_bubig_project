/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DM_IMU_H
#define DM_IMU_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h7xx_hal.h"
#include "bsp_can.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies CAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies CAN recieved identifier */

}IMU_CANFrameInfo_typedef;

typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;
	IMU_CANFrameInfo_typedef CANFrame;
}DM_IMU_Info_Typedef;


void DM_IMU_Info_Update(uint8_t* pData);
void IMU_RequestData(FDCAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg);
extern DM_IMU_Info_Typedef DM_IMU_Info;


#endif

