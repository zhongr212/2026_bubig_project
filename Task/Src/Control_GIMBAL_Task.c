//#include "Control_GIMBAL_Task.h"
//#include "Control_Task.h"
//#include "bsp_can.h"
//#include "cmsis_os.h"
//#include "Motor.h"
//#include "pid.h"
//#include "usart.h"
//#include "remote_control.h"
//#include "math.h"
//#include "arm_math.h" 
//#include "Bmi088.h" 
//#include "Quaternion.h"
//#include "bsp_uart.h"
//#include "INS_Task.h"
//#include "lpf.h"
//#include "bsp_rs485.h"
//#include "fdcan.h"
//#include "RLS.h"
//#include "Config.h"
//#include "bsp_uart.h"
//#include "stdio.h"
//#include "bsp_uart.h"
//#include "Config.h"
//#include "main.h"
//#include "tim.h"
//#include "gpio.h"
//#include "Referee_System.h"
//#include "ramp.h"
//void Yaw_Cal();
//void Init_yaw();
//void Yaw_off();
//void pid_yawprogram_select(int mode);
//void fire_choose();
//float remoteSensitivity[2] = {0.5}; 
//float yaw_pid_program[3][7]={
//[0]={30,0,0,0,3,15000},
//[1]={140,1,0,0,2,22000},
////12.5,0,0,0,3000,17000
////238,1.8,0,0,2000,17000

//};
//PID_Info_TypeDef yaw_pid[2];
//test_program_Info_Typedef test_program_Info; 
//float fire_mode;

//void Control_GIMBAL_Task(void const * argument)
//{
//	
//	Init_yaw();
//  /* USER CODE BEGIN Control_GIMBAL_Task */
//  /* Infinite loop */
//  for(;;)
//  {
//			  if(Chassis_Info.mode.symbol!=OUT)
//	{
//	 Yaw_Cal();
//	
//  } 
//		else
//			Yaw_off();
//		
//    osDelay(1);
//  }
//  /* USER CODE END Control_GIMBAL_Task */
//}

//void Init_yaw()
//{

//	
//	
//  PID_Init(&yaw_pid[0],PID_POSITION,yaw_pid_program[0]);        
//	PID_Init(&yaw_pid[1],PID_POSITION,yaw_pid_program[1]);   

//YAW_Info.target=UART_485_COMMUNICATE.yaw_angle;



//}
//float zero_a[2];
//float yawadd=0.05;
//float yawadd_sum=0;
//void Yaw_Cal()
//{
//if(Chassis_Info.mode.symbol==Normal){
//	
//	pid_yawprogram_select(Normal);
//	YAW_Info.target-=((float)((remote_ctrl.rc.ch[0]/660.f)*remoteSensitivity[0]));
//		if(YAW_Info.target>=180)
//		{
//		YAW_Info.target-=360;
//		}
//else
//    if(YAW_Info.target<=-180)
//   {

//    YAW_Info.target+=360;
//    }	


//		
//	}

//	else
//	if(Chassis_Info.mode.symbol==Auto_aim)
//	{
//	
//	
//						if(UART_485_COMMUNICATE.distance!=-1&&UART_485_COMMUNICATE.distance!=0)
//					{

//							YAW_Info.target=UART_485_COMMUNICATE.Minipc_yaw;
//               pid_yawprogram_select(Auto_aim);
//				}
//					else
//							if(UART_485_COMMUNICATE.Minipc_yaw==0||UART_485_COMMUNICATE.distance==-1){
//															YAW_Info.target=UART_485_COMMUNICATE.last_yaw_minipc;				
//	                       }
//							
//				
//				if(UART_485_COMMUNICATE.distance!=-1&&UART_485_COMMUNICATE.distance!=0)
//				UART_485_COMMUNICATE.last_yaw_minipc=UART_485_COMMUNICATE.Minipc_yaw;
//	
////////////////////////////////////////////////////////////////////////////////////////////
//	
//////////////////////////////////////////////////////////////////////////////////////	
////		pid_yawprogram_select(Normal);
////	YAW_Info.target-=((float)((remote_ctrl.rc.ch[0]/660.f)*remoteSensitivity[0]));
////		if(YAW_Info.target>=180)
////		{
////		YAW_Info.target-=360;
////		}
////else
////    if(YAW_Info.target<=-180)
////   {

////    YAW_Info.target+=360;
////    }	
//	
//	
//	
//	
//	}
//	else
//		if(Chassis_Info.mode.symbol==Calibrate)
//	YAW_Info.target=UART_485_COMMUNICATE.yaw_angle;
//else
//		if(Chassis_Info.mode.symbol==Independent)
//	{
//if(Chassis_Info.need_clibrate==2)
//{
//		if((UART_485_COMMUNICATE.distance!=-1)&&(UART_485_COMMUNICATE.distance!=0))
//					{

//							YAW_Info.target=UART_485_COMMUNICATE.Minipc_yaw;
//				}
//					else
//						if((UART_485_COMMUNICATE.Minipc_yaw==0)||(UART_485_COMMUNICATE.distance==-1))
//						{
//							test_program_Info.yaw_zero_angle[0]=0;
//            	YAW_Info.target=test_program_Info.yaw_zero_angle[0]+yawadd_sum;
//		          if(YAW_Info.target>(test_program_Info.yaw_zero_angle[0]+180))
//	              	{
//		                yawadd=-0.05;
//	
//	                   	}		
//	                	else
//		                  	if(YAW_Info.target<(test_program_Info.yaw_zero_angle[0]-180))
//			                  {
//			
//		                     	yawadd=0.05;
//		                     	}
//		                      yawadd_sum+=yawadd;

//							
//							
//						
//						
//						
//						}
//						
//						
//						
//		
//		}
//else
//	if(Chassis_Info.need_clibrate==1)
//	{
//	
//	
//	
//	
//	}
//					
//					
//					
//					
//					
//					
//					
//					
//					
//					
//					
//					
////				if(UART_485_COMMUNICATE.fire==1)fire_choose();
////        else Chassis_Info.shoot.target=0;
//				
//				if(UART_485_COMMUNICATE.distance!=-1&&UART_485_COMMUNICATE.distance!=0)
//				UART_485_COMMUNICATE.last_yaw_minipc=UART_485_COMMUNICATE.Minipc_yaw;
//	/////////////////////////////////
//	Chassis_Info.shoot.target=0;
//	/////////////////////////////////

//	
//	}	
//	
//	
//YAW_Info.yaw_error[0]=YAW_Info.target-UART_485_COMMUNICATE.yaw_angle;
//if(YAW_Info.yaw_error[0]>=180)
//	YAW_Info.yaw_error[0]-=360;
//else
//	if(YAW_Info.yaw_error[0]<=-180)
//		YAW_Info.yaw_error[0]+=360;

//if(Chassis_Info.mode.symbol!=Auto_aim)
//	YAW_Info.yaw_bf=0;







//	f_PID_Calculate(&yaw_pid[0],YAW_Info.yaw_error[0],0);
//  YAW_Info.output=f_PID_Calculate(&yaw_pid[1],yaw_pid[0].Output,UART_485_COMMUNICATE.Yaw_Gyro);

//}

//void pid_yawprogram_select(int mode)
//{
//   if(mode==Auto_aim)
//	 {
//	 yaw_pid[0].param.kp=50;
//	 yaw_pid[0].param.ki=0;
//	 yaw_pid[0].param.kd=10;
//	 yaw_pid[0].param.Deadband=0;
//	 yaw_pid[0].param.limitIntegral=2;
//	 yaw_pid[0].param.limitOutput=22000;
//		 
//	 yaw_pid[1].param.kp=300;
//	 yaw_pid[1].param.ki=0;
//	 yaw_pid[1].param.kd=50;
//	 yaw_pid[1].param.Deadband=0;
//	 yaw_pid[1].param.limitIntegral=2;
//	 yaw_pid[1].param.limitOutput=22000;		 
//		 
//		 
//		 
//	 }

//else
//	if(mode==Normal)
//	{
//	
//	
//	 yaw_pid[0].param.kp=30;
//	 yaw_pid[0].param.ki=0;
//	 yaw_pid[0].param.kd=0;
//	 yaw_pid[0].param.Deadband=0;
//	 yaw_pid[0].param.limitIntegral=5;
//	 yaw_pid[0].param.limitOutput=17000;
//		 
//	 yaw_pid[1].param.kp=140;
//	 yaw_pid[1].param.ki=0;
//	 yaw_pid[1].param.kd=0;
//	 yaw_pid[1].param.Deadband=0;
//	 yaw_pid[1].param.limitIntegral=5;
//	 yaw_pid[1].param.limitOutput=22000;		 
//	
//	
//	
//	
//	
//	}






// }
// void fire_choose()
// {
// if(Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat>=0&&Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat<=400)
//    Chassis_Info.shoot.Heat.Heat_Ctrl_Err= Referee_System_Info.robot_status.shooter_barrel_heat_limit - Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat;
	
	
// 	if(Chassis_Info.shoot.Heat.Shoot_Count==0){
	
// 	Chassis_Info.shoot.Heat.Shoot_Time = (Chassis_Info.shoot.Heat.Heat_Ctrl_Err + 1 * Referee_System_Info.robot_status.shooter_barrel_cooling_value)* 10;
//        VAL_LIMIT(Chassis_Info.shoot.Heat.Shoot_Time, 200, 5600);
		
// 	if(Chassis_Info.shoot.Heat.Heat_Ctrl_Err < 100){
	
// 	Chassis_Info.shoot.Heat.Tigger_Speed = (10 * Chassis_Info.shoot.Heat.Heat_Ctrl_Err - Referee_System_Info.robot_status.shooter_barrel_cooling_value - 3 * 10) / (10 * (Chassis_Info.shoot.Heat.Shoot_Time / 100.f)) + Referee_System_Info.robot_status.shooter_barrel_cooling_value / 10;
	
// 	}else {
	
// 	Chassis_Info.shoot.Heat.Tigger_Speed = (10 * Chassis_Info.shoot.Heat.Heat_Ctrl_Err - Referee_System_Info.robot_status.shooter_barrel_cooling_value - 5 * 10) / (10 * (Chassis_Info.shoot.Heat.Shoot_Time / 100.f)) + Referee_System_Info.robot_status.shooter_barrel_cooling_value / 10;
	
// 	}
	
	
// 	}else if(0 < Chassis_Info.shoot.Heat.Shoot_Count && Chassis_Info.shoot.Heat.Shoot_Count < Chassis_Info.shoot.Heat.Shoot_Time){
	
// 	Chassis_Info.shoot.Heat.BulletFeed = (Chassis_Info.shoot.Heat.Tigger_Speed * 2 * PI / 8);
	
// 	VAL_LIMIT(Chassis_Info.shoot.Heat.BulletFeed, 0.0f, 25.0f);
	
// 	}else
//    {
//        	Chassis_Info.shoot.Heat.BulletFeed = ((Referee_System_Info.robot_status.shooter_barrel_cooling_value / 10) * 2 * PI / 10);
//        VAL_LIMIT(Chassis_Info.shoot.Heat.BulletFeed, 0.0f, 25.0f);
//    }
		
// 		 if (Chassis_Info.shoot.Heat.Shoot_Count<Chassis_Info.shoot.Heat.Shoot_Time)
//    {
//        Chassis_Info.shoot.Heat.Shoot_Count++;
//    }
//    Chassis_Info.shoot.Heat.Last_Shoot_time = Chassis_Info.shoot.Heat.Shoot_Time;	
	
// 	if (Chassis_Info.shoot.Heat.Heat_Ctrl_Err >= 50)
//    {
//        if (Chassis_Info.shoot.Heat.Shoot_Count >= Chassis_Info.shoot.Heat.Shoot_Time)
//        {
//            Chassis_Info.shoot.Heat.Shoot_Count = 0;
//        }
//    }
//    else if (Chassis_Info.shoot.Heat.Heat_Ctrl_Err <= 40)
//    {
//       Chassis_Info.shoot.Heat.Shoot_Count = Chassis_Info.shoot.Heat.Last_Shoot_time;
//    }
// 		else
// 	      if(Chassis_Info.shoot.Heat.Heat_Ctrl_Err <= 35)
// 		       Chassis_Info.shoot.Heat.BulletFeed=0;
//             //ÈÈÁ¿¿ØÖÆ
	
// Chassis_Info.shoot.Heat.BulletFeed =Chassis_Info.shoot.Heat.BulletFeed * -440.f;

		
// 	//	Control_Info.Tigger.Tigger_Speed = 0;
		
		
// }




//void Yaw_off()
//{
//	YAW_Info.target=UART_485_COMMUNICATE.yaw_angle;
//  YAW_Info.output=0;




//}

