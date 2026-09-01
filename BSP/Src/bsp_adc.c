/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_adc.c
  * @brief          : bsp adc functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the adc
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"
#include "adc.h"
#include "tim.h"
/**
 * @brief ADC sampling voltage array
 */
__attribute__((section (".RAM_D1"))) uint16_t ADC_Voltage_Val[2];
 	VoltageMonitor_Info_Typdef Voltage_Monitor ;
/**
  * @brief  Configures the ADC. 
  * @param  None
  * @retval None
  */
void BSP_ADC_Init(void){
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Voltage_Val,2);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER get current voltage. 
  * @param  None
  * @retval Voltage
  */
float GetBatteryVoltage(void){

  float Voltage;
	Voltage = (ADC_Voltage_Val[0]*3.3f/65535)*11.0f;
	return Voltage;
   
}   


 /**
  * @brief 初始化电压监测器
  * @param monitor 电压监测器指针
  * @param low_threshold 低电量阈值(V)
  * @param critical_threshold 危险电量阈值(V)
  */
 void InitVoltageMonitor(VoltageMonitor_Info_Typdef *monitor, float low_threshold, float critical_threshold)
 {
     monitor->low_threshold = low_threshold;
     monitor->critical_threshold = critical_threshold;
     monitor->last_check_time = 0;
     monitor->status = VOLTAGE_NORMAL;
 }

 /**
  * @brief 更新电压监测器状态
  * @param monitor 电压监测器指针
  * @details 定期检查电池电压，并更新状态
  * @note 需要在主循环中定期调用
  */
 void UpdateVoltageMonitor(VoltageMonitor_Info_Typdef *monitor)
 {
     uint32_t current_time = HAL_GetTick();

     // 每隔1000ms检查一次电压
     if (current_time - monitor->last_check_time >= VOLTAGE_CHECK_INTERVAL)
     {
         monitor->last_check_time = current_time;
			   monitor->Last_status = monitor->status;
         monitor->current_voltage = GetBatteryVoltage();
         
         // 更新电压状态
         if (monitor->current_voltage > monitor->critical_threshold)
         {
             monitor->status = VOLTAGE_CRITICAL;
         }
         else if (monitor->current_voltage < monitor->low_threshold)
         {
             monitor->status = VOLTAGE_LOW;
         }
         else
         {
             monitor->status = VOLTAGE_NORMAL;
         }
     }
 }

 /**
  * @brief 获取当前电压状态
  * @param monitor 电压监测器指针
  * @return 电压状态枚举值
  */
 Voltage_Status_e GetVoltageStatus(VoltageMonitor_Info_Typdef *monitor)
 {
	if(monitor->Last_status!=monitor->status)
		{
				monitor->Last_status=monitor->status;
return monitor->status;
	}
	else
	{
		monitor->status=VOLTAGE_NORMAL;
return monitor->status;
	}

 }

 /**
  * @brief 获取当前电压值
  * @param monitor 电压监测器指针
  * @return 当前电压值(V)
  */
 float GetCurrentVoltage(VoltageMonitor_Info_Typdef *monitor)
 {
     return monitor->current_voltage;
 }