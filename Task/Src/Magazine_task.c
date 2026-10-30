/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Magazine_task.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "PID.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "Referee_System.h"
#include "rc_vtm_control.h"



static void Control_Init(Control_Magazine_Info_Typedef *Control_Magazine_Info);
static void Control_Measure_Update(Control_Magazine_Info_Typedef *Control_Magazine_Info);
static void Control_Target_Update(Control_Magazine_Info_Typedef *Control_Magazine_Info);
static void Control_Magazine_Info_Update(Control_Magazine_Info_Typedef *Control_Magazine_Info);
static void Control_Mode(Control_Magazine_Info_Typedef *Control_Magazine_Info);
float Heat_control(uint16_t Magazine_Velocity);
	
Control_Magazine_Info_Typedef Control_Magazine_Info;
//                                        KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float Magazine_Angle_PID_Param[7] = {1.6f,0.4f,0.1f, 0.f,  0.f,      1.f,  100.f};
//                                          KP    KI   KD Alpha Deadband  I_MAX   Output_MAX
static float Magazine_Velocity_PID_Param[7] = {200.0f, 0.5f, 0.06f, 0.01f, 0.1f, 50.f, 8000.f};

static float Shoot_Velocity_PID_Param[7] = {70.0f, 0.0f, 0.05f, 0.0f, 0.0f, 0.f, 20000.f};

PID_Info_TypeDef Magazine_Angle_PID;
PID_Info_TypeDef Magazine_Velocity_PID;
PID_Info_TypeDef Shoot0_Velocity_PID;
PID_Info_TypeDef Shoot1_Velocity_PID;
float Magazine_ZeroPoint_MotorAngle;             // Midpoint for lower yaw motor
uint16_t time_flag_1;
char frist_flag;
char pull,last_pull;
void Magazine_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
	vTaskDelay(100);
	TickType_t Control_Task_SysTick = 0;
  
	Control_Init(&Control_Magazine_Info);
	pull = 0;
	frist_flag = 1;
 /* Infinite loop */
	for(;;)
  {
	 	Control_Task_SysTick = osKernelSysTick();
		Control_Measure_Update(&Control_Magazine_Info);
		Control_Mode(&Control_Magazine_Info);
		Control_Target_Update(&Control_Magazine_Info);
		Control_Magazine_Info_Update(&Control_Magazine_Info);
		USART_Vofa_Justfloat_Transmit(Control_Magazine_Info.Target.Magazine_Velocity,Control_Magazine_Info.Measure.Magazine_Velocity,0); 
		osDelay(1);
  }
}
  /* USER CODE END Control_Task */

static void Control_Init(Control_Magazine_Info_Typedef *Control_Magazine_Info){


  PID_Init(&Magazine_Velocity_PID,PID_POSITION,Magazine_Velocity_PID_Param);
	PID_Init(&Magazine_Angle_PID,PID_ANGLE,Magazine_Angle_PID_Param);
  PID_Init(&Shoot0_Velocity_PID,PID_POSITION,Shoot_Velocity_PID_Param);
  PID_Init(&Shoot1_Velocity_PID,PID_POSITION,Shoot_Velocity_PID_Param);
	
  Control_Magazine_Info->SendValue[0] = 0;
  Control_Magazine_Info->SendValue[1] = 0;
  Control_Magazine_Info->SendValue[2] = 0;
  Control_Magazine_Info->SendValue[3] = 0;
  Magazine_ZeroPoint_MotorAngle = Magazine_Motor.Data.Angle;
	osDelay(1000);
}



static void Control_Measure_Update(Control_Magazine_Info_Typedef *Control_Magazine_Info){
	Control_Magazine_Info->Measure.Magazine_Angle = Magazine_Motor.Data.Angle-Magazine_ZeroPoint_MotorAngle;
  Control_Magazine_Info->Measure.Magazine_Velocity = Magazine_Motor.Data.Velocity/96.0f;//*4.0f/30.0f;// 单位：发
	

	Control_Magazine_Info->Measure.Shoot0_Velocity = Shoot_Motor[0].Data.Velocity;
	Control_Magazine_Info->Measure.Shoot1_Velocity = Shoot_Motor[1].Data.Velocity;
  
}

static void Control_Mode(Control_Magazine_Info_Typedef *Control_Magazine_Info)
{
	last_pull = pull;
	pull = remote_ctrl.rc.s[0];
	if(frist_flag == 1)
	{
//	if((last_pull == pull)&&(pull == 3))
//	{
//		Control_Magazine_Info->Type = SHOOT_ENABLE;
//		time_flag_1 = osKernelSysTick();
//	do
//	{Control_Magazine_Info->SendValue[0] = 0;
//	Control_Magazine_Info->Type = SHOOT_SINGLE;
//	}//测试后更改
//	while((Magazine_Motor.Data.Current<=1000)||(osKernelSysTick() - time_flag_1 <=2000));//卡限位，没测

//	Control_Magazine_Info->Type = SHOOT_BACK;
//	}
	
	frist_flag = 0;
	}
	else
	{
		if((last_pull == pull)&&((pull == 3)))
		{
			Control_Magazine_Info->Type = SHOOT_SLEEP;
		}
	  if((last_pull == 3)&&(pull == 1))
		{
			Control_Magazine_Info->Type = SHOOT_SINGLE;
		}
		if((last_pull == 3)&&(pull == 2)||(vtm_rc_data.mouse.left))
		{
			Control_Magazine_Info->Type = SHOOT_FIRE;
		}
		if((last_pull == 1)&&(pull == 1))
		{
		Control_Magazine_Info->Type = SHOOT_SINGLE_WAIT;
		}
	}
	
//				Control_Magazine_Info->Type = SHOOT_FIRE;
}
static void Control_Target_Update(Control_Magazine_Info_Typedef *Control_Magazine_Info){
	
//Control_Magazine_Info->Target.Magazine_Angle += 0.044f*Control_Magazine_Info->fireRate;//后为射频3，0.044是固定奇妙系数
if(Control_Magazine_Info->Type == SHOOT_SINGLE)
{
Control_Magazine_Info->Target.Magazine_Angle = Control_Magazine_Info->Measure.Magazine_Angle;
Control_Magazine_Info->Target.Magazine_Angle+=45;
}
else if(Control_Magazine_Info->Type == SHOOT_FIRE)
{
Control_Magazine_Info->Target.Magazine_Velocity=30;
Control_Magazine_Info->Target.Shoot0_Velocity = -1*Control_Magazine_Info->fireSpeed;
Control_Magazine_Info->Target.Shoot1_Velocity =  1*Control_Magazine_Info->fireSpeed;
}
else if(Control_Magazine_Info->Type == SHOOT_BACK)
{
Control_Magazine_Info->Target.Magazine_Angle = Control_Magazine_Info->Measure.Magazine_Angle;
Control_Magazine_Info->Target.Magazine_Angle-=40;
}


}



static void Control_Magazine_Info_Update(Control_Magazine_Info_Typedef *Control_Magazine_Info){
  if((Control_Magazine_Info->Type == SHOOT_SINGLE)||(Control_Magazine_Info->Type == SHOOT_SINGLE_WAIT))
  {
	PID_Calculate(&Magazine_Angle_PID,    Control_Magazine_Info->Target.Magazine_Angle, Control_Magazine_Info->Measure.Magazine_Angle);
  PID_Calculate(&Magazine_Velocity_PID, Magazine_Angle_PID.Output,                   Control_Magazine_Info->Measure.Magazine_Velocity);
	}
	else if(Control_Magazine_Info->Type == SHOOT_FIRE)
	{
	PID_Calculate(&Magazine_Velocity_PID, Control_Magazine_Info->Target.Magazine_Velocity,Control_Magazine_Info->Measure.Magazine_Velocity);
	}
  PID_Calculate(&Shoot0_Velocity_PID, Control_Magazine_Info->Target.Shoot0_Velocity,   Control_Magazine_Info->Measure.Shoot0_Velocity);	
  PID_Calculate(&Shoot1_Velocity_PID, Control_Magazine_Info->Target.Shoot1_Velocity,   Control_Magazine_Info->Measure.Shoot1_Velocity);	

	Control_Magazine_Info->SendValue[0] = (int16_t)(Magazine_Velocity_PID.Output);
	Control_Magazine_Info->SendValue[1] = (int16_t)(0);
	Control_Magazine_Info->SendValue[2] = (int16_t)(Shoot0_Velocity_PID.Output);
	Control_Magazine_Info->SendValue[3] = (int16_t)(Shoot1_Velocity_PID.Output);
//	USART_Vofa_Justfloat_Transmit(Control_Magazine_Info->Target.Magazine_Angle,Control_Magazine_Info->Measure.Magazine_Angle,0);
	if(Control_Magazine_Info->Type == SHOOT_SLEEP){
		Control_Magazine_Info->SendValue[0] = (int16_t)0;
		Control_Magazine_Info->SendValue[1] = (int16_t)0;
	}
}

float Heat_control(uint16_t Magazine_Velocity)
{
	//剩余热量
	uint16_t surplus_heat = Referee_System_Info.robot_status.shooter_barrel_heat_limit - Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat;
	uint16_t recover_heat = Referee_System_Info.robot_status.shooter_barrel_cooling_value;
	float recover_v = recover_heat/1.0;//恢复热量
	
	
	if(surplus_heat <=200)
	{
		Magazine_Velocity =(uint16_t)((Magazine_Velocity - recover_v)/150.0*surplus_heat+(4*recover_v-Magazine_Velocity)/3.0);
	}
	else if(surplus_heat<=50)
	{
		Magazine_Velocity = recover_v;
	}
	else if(surplus_heat<=20)
	{
		Magazine_Velocity = 0;
	}
	return Magazine_Velocity;
}