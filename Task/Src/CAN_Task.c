/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "CRC.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "Control_Task.h"
#include "Magazine_task.h"
#include "Motor.h"
#include "Control_Gaimbal_task.h"
#include "DM_imu.h"
#include "fdcan.h"

uint8_t tick_ms;
uint32_t freq_1k;



uint8_t cantx_data_chassis[8];
uint8_t cantx_data_gimbal[8];
uint8_t cantx_data_Supercap[8];
extern UART_HandleTypeDef huart1; 
Sentry_Tx_Referee_t Sentry_Tx_Referee;
sentry_status_t sentry_status; 
static int float_to_uint(float x, float x_min, float x_max, int bits);
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD);
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */

 void CAN_Task(void const * argument)
{
	Sentry_Tx_Referee.header.SOF=0xA5;
  Sentry_Tx_Referee.header.length=10;
  Sentry_Tx_Referee.data.data_cmd_id=0x0120;	
	Sentry_Tx_Referee.data.receiver_id=0x8080;
	Sentry_Tx_Referee.cmd_id=0x0301;
	Sentry_Tx_Referee.data.sender_id=107;	//blue/////red 7
	Sentry_Tx_Referee.data.sentry_cmd=1;
	
 uint8_t CAN_FD_Tx_Buffer[64] = { 0 };
  TickType_t CAN_Task_SysTick = 0;
	for(;;)
  {
  CAN_Task_SysTick = osKernelSysTick();		

		FDCAN1_TxFrame.Header.Identifier=0x200;
	  cantx_data_chassis[0]=(uint8_t)(Chassis_Info.SendValue[0]>>8);
		cantx_data_chassis[1]=(uint8_t)(Chassis_Info.SendValue[0]);
		cantx_data_chassis[2]=(uint8_t)(Chassis_Info.SendValue[1]>>8);
		cantx_data_chassis[3]=(uint8_t)(Chassis_Info.SendValue[1]);
		cantx_data_chassis[4]=(uint8_t)(Chassis_Info.SendValue[2]>>8);
		cantx_data_chassis[5]=(uint8_t)(Chassis_Info.SendValue[2]);
		cantx_data_chassis[6]=(uint8_t)(Chassis_Info.SendValue[3]>>8);
		cantx_data_chassis[7]=(uint8_t)(Chassis_Info.SendValue[3]);
	USER_CAN_TxMessage(&FDCAN1_TxFrame,Motor_chassis,cantx_data_chassis); 
	 	
		freq_1k = osKernelSysTick();
		
		for(;;)
		{
			tick_ms++;
			if(tick_ms%3==0)
			{
				IMU_RequestData(&hfdcan3,0x01,3);
			}
			else if(tick_ms%2==0)
			{
				IMU_RequestData(&hfdcan3,0x01,2);
			}
			else if(tick_ms%1==0)
			{
				IMU_RequestData(&hfdcan3,0x01,1);
			}
			osDelayUntil(&freq_1k, 1);
		}
		
		
	 FDCAN2_TxFrame.Header.Identifier=0x1FF;
	 Magazine_Motor.CANFrame.TxStdId=0x1ff;
	 FDCAN2_TxFrame.Data[4] = (uint8_t)(Control_Magazine_Info.SendValue[0]>>8);	  
	 FDCAN2_TxFrame.Data[5] = (uint8_t)(Control_Magazine_Info.SendValue[0]);	
	 USER_CAN_TxMessage(&FDCAN2_TxFrame,&Magazine_Motor,FDCAN2_TxFrame.Data); 

		DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_Yaw_Motor, 0, 0, 0, 0, Control_Gimbal_Info.SendValue[1] ); // YawÖá
//	  if (Control_Gimbal_Info.Type == GIMBAL_SLEEP)
//		DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Disable);
//		else   if  (Control_Gimbal_Info.Type != GIMBAL_SLEEP)               //((Control_Info.Control_Mode != Control_Info.Gimbal_Last_Mode) &&
		DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Enable);

		osDelay(1);
  }
 
}


/**
  * @brief  CAN Transmit DM motor Information
  * @param  *FDCAN_TxFrame  pointer to the FDCAN_TxFrame_TypeDef.
  * @param  *DM_Motor  pointer to the DM_Motor
  * @param  Postion Velocity KP KD Torgue: Target
  * @retval None
  */
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque){
	
   if(DM_Motor->Control_Mode == MIT){
		 
		 uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
		 
		 Postion_Tmp  =  float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16) ;
		 Velocity_Tmp =  float_to_uint(Velocity,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
		 Torque_Tmp   =  float_to_uint(Torque,  -DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
		 KP_Tmp = float_to_uint(KP,0,500,12);
		 KD_Tmp = float_to_uint(KD,0,5,12);
		
		 FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxStdId;
		 
		 FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
		 FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
		 FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
		 FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
		 FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
		 FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
		 FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
		 FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);

	}else if(DM_Motor->Control_Mode == POSITION_VELOCITY){
	
		 uint8_t *Postion_Tmp,*Velocity_Tmp;
		
		 Postion_Tmp  = (uint8_t*) & Postion;
		 Velocity_Tmp = (uint8_t*) & Velocity;
		
	   FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxStdId + 0x100;
		
		 FDCAN_TxFrame->Data[0] = *(Postion_Tmp);
		 FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
		 FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
		 FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
	   FDCAN_TxFrame->Data[4] = *(Velocity_Tmp);
		 FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
		 FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
		 FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
		
	}else if(DM_Motor->Control_Mode == VELOCITY){
	
	  uint8_t *Velocity_Tmp;
		Velocity_Tmp = (uint8_t*) & Velocity;
		
    FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxStdId + 0x200;
		
		FDCAN_TxFrame->Data[0] = *(Velocity_Tmp);
		FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
		FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
		FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
		FDCAN_TxFrame->Data[4] = 0;
 		FDCAN_TxFrame->Data[5] = 0;
		FDCAN_TxFrame->Data[6] = 0;
		FDCAN_TxFrame->Data[7] = 0;

	}
	 
	  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);

}

void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD){

	 FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxStdId;
  	
	 FDCAN_TxFrame->Data[0] = 0xFF;
   FDCAN_TxFrame->Data[1] = 0xFF;
 	 FDCAN_TxFrame->Data[2] = 0xFF;
	 FDCAN_TxFrame->Data[3] = 0xFF;
	 FDCAN_TxFrame->Data[4] = 0xFF;
	 FDCAN_TxFrame->Data[5] = 0xFF;
	 FDCAN_TxFrame->Data[6] = 0xFF;
	
	 switch(CMD){
		 
		  case Motor_Enable :
	        FDCAN_TxFrame->Data[7] = 0xFC; 
	    break;
      
			case Motor_Disable :
	        FDCAN_TxFrame->Data[7] = 0xFD; 
      break;
      
			case Motor_Save_Zero_Position :
	        FDCAN_TxFrame->Data[7] = 0xFE; 
			break;
			
			default:
	    break;   
	}
	
   USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);

}


static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
	
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
	
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
