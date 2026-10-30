
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Detect_Task.c
 * @brief          : Detect task
 * @author         : Yan Yuanbin
 * @date           : 2023/04/27
 * @version        : v1.0
 ******************************************************************************
 * @attention      : None
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_adc.h"
#include "Motor.h"
#include "Warning_Task.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
static void System_SelfCheck();

	/* USER CODE BEGIN Header_Warning_Task */
/**
* @brief Function implementing the StartWarning_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Warning_Task */
 void Warning_Task(void const * argument)
{
  /* USER CODE BEGIN Warning_Task */
	
	TickType_t systick = 0;

  InitVoltageMonitor(&Voltage_Monitor, 20.2f, 26.0f);
	BSP_ADC_Init();
	
  /* Infinite loop */
  for(;;)
  {
		systick = osKernelSysTick();
		Remote_Message_Moniter(&remote_ctrl);
		updateMusicPlayer(&music_player);
    UpdateVoltageMonitor(&Voltage_Monitor);
    System_SelfCheck();
	
    osDelay(1);
  }
  /* USER CODE END Warning_Task */
}


//Motor_Status_e GetMortorState(DJI_Motor_Info_Typedef *DJI_Motor)
//{
//		  if (DJI_Motor->Data.Current==DJI_Motor->Data.Last_Current)
//  {
//    DJI_Motor->ERRORHandler.CAN_ErrorCount++;

//    if (DJI_Motor->ERRORHandler.CAN_ErrorCount > 500)
//    {
//      DJI_Motor->ERRORHandler.Status = MOTOR_CAN_OFFLINE;
//      DJI_Motor->ERRORHandler.CAN_ErrorCount = 0;
//    }
//  }
//  else
//  {
//    DJI_Motor->ERRORHandler.CAN_ErrorCount = 0;
//  }
//    DJI_Motor->Data.Last_Current=DJI_Motor->Data.Current;
//	
//	

//	if(DJI_Motor->ERRORHandler.Last_status!=DJI_Motor->ERRORHandler.Status)
//		{
//				DJI_Motor->ERRORHandler.Last_status=DJI_Motor->ERRORHandler.Status;
//return DJI_Motor->ERRORHandler.Status;
//	}
//	else
//	{
//		DJI_Motor->ERRORHandler.Status=MOTOR_ERROR_NONE;
//return MOTOR_ERROR_NONE;
//	}
//}



void System_SelfCheck()
{

    switch (GetVoltageStatus(&Voltage_Monitor))
    {
    case VOLTAGE_NORMAL:
      break;
    case VOLTAGE_LOW:
      LowVoltage_Music();
      break;
    case VOLTAGE_CRITICAL:
      LowVoltage_Music();
      break;
	  }
	
//switch(GetMortorState(&Super_Cap))
//{
//	case MOTOR_ERROR_NONE:
//		break;
//	case MOTOR_CAN_OFFLINE:
//	{
//CAN1_Lost_Music();
//	}
//		break;
//	case MOTOR_OVER_TEMPERATURE:
//HighHeat_Music();
//	break;
//}
		

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */