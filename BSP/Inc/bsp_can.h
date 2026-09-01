/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "Motor.h"
/**
 * @brief The structure that contains the Information of FDCAN Transmit.
 */
typedef struct{
		FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of FDCAN Receive.
 */
typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
}FDCAN_RxFrame_TypeDef;

typedef struct 
{
	
		struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	 float  rc_lost; 
	
	
} REMOTO_CTRL_Typedef;
extern REMOTO_CTRL_Typedef REMOTO_CTRL;

typedef struct 
{
	
float Yaw_Angle;
float Yaw_Gyro;
	
} Gimbal_Typedef;
extern Gimbal_Typedef Gimbal;

typedef struct 
{
	uint8_t Yaw_Angle_test[4];
	uint8_t Yaw_Gyro_test[4];

} UART_Typedef;
extern UART_Typedef UART;


typedef struct{
  //导航接收
uint8_t Header;
  float Vx;
  float Vy;
  float Vz;
	float Vision_Yaw;
	float Distance;
	float Spon_Speed;
   uint8_t Vx_test[4];
   uint8_t Vy_test[4];
	 uint8_t Vz_test[4];
	 uint8_t Spin_test[4];
	 uint8_t heat_outpost_int[4];
	uint8_t Vision_Grap;
	 uint8_t Spin_Switch;
	uint8_t Vision_Yaw_test[4];
	
  //决策接受
  uint8_t Fire;  //0不开火，1开火
  uint8_t Mode;  //0，自检 1自动
	uint8_t Auto_to_Vision;
	uint8_t start;
	uint8_t Fire_Switch_Spin;
	
}MiniPc_Receive_Auto_Typedef;

//电容相关信息
typedef struct
{
    uint8_t errorCode;        // 错误码
    float chassisPower;       // 实际底盘功率
    uint16_t chassisPowerLimit; // 功率上限
    uint8_t capEnergy;        // 电容能量(0-255)
} Super_C_Msg_t;


/* Externs ------------------------------------------------------------------*/
extern  FDCAN_TxFrame_TypeDef   FDCAN1_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN2_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN3_TxFrame;
extern  MiniPc_Receive_Auto_Typedef  MiniPc_Receive_Auto;
extern void  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
extern void USER_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DJI_Motor_Info_Typedef*DJI_MOTOR,uint8_t data[8]);
extern void BSP_FDCAN_Init(void);

	   
#endif
