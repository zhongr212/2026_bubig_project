#include "cmsis_os.h"
#include "arm_math.h"
#include "Control_Gaimbal_task.h"
#include "Motor.h"
#include "INS_Task.h"
#include "DM_IMU.h"
#include "Remote_Control.h"
#include "bsp_pwm.h"
#include "Kalman_Filter.h"
#include "PID.h"
#include "bsp_uart.h"
#include "rc_vtm_control.h"
#include  "slidingmode.h"

//#include "aiming.h"

/* Internal Control Function Prototypes --------------------------------------*/
static void Control_Init(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Measure_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Target_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Mode_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Gimbal_Info_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);

/* Gimbal Control Mode Function Prototypes -----------------------------------*/
void Control_Gimbal_Remote(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);


/* Global Variables ----------------------------------------------------------*/
Control_Gimbal_Info_Typedef Control_Gimbal_Info;  // Main gimbal control information structure

/* PID Controller Parameters -------------------------------------------------*/
// Pitch axis control parameters [KP, KI, KD, Alpha, Deadband, I_MAX, Output_MAX, I_Variable_Speed_A ,I_Variable_Speed_B ,KF, ]
static float Pitch_Angle_PID_Param[10] = {6.5f, 0.007f, 18.0f, 0.0f, 0.0f, 5.f, 7.5f, 0.0272f, 0.4363f, 0.0f};
static float Pitch_Velocity_Param[10] = {11000.0f, 48.0f, 3500.0f, 0.0f, 0.0f, 20000.0f, 25000.f, 2.5f, 0.25f, 0};


// Lower yaw axis control parameters     p      i     d    a     d     im   om     A      B        KF
static float Yaw_Angle_PID_Param[10] = {1.5f, 0.f, 0.0f, 0.0f, 0.0f, 3.f, 20.0f, 0.1f, 0.4363f, 0.0f};
static float Yaw_Velocity_PID_Param[10] = {0.09f, 0.f, 0.20f, 0.0f, 0.0f, 0.5f, 1.0f, 0.5f, 0.25f, 0};

/* PID Controller Instances --------------------------------------------------*/
PID_Info_TypeDef Pitch_Angle_PID;      // Pitch angle position controller
PID_Info_TypeDef Pitch_Velocity_PID;   // Pitch velocity controller

PID_Info_TypeDef Yaw_Angle_PID;   // Lower yaw angle position controller
PID_Info_TypeDef Yaw_Velocity_PID;// Lower yaw velocity controller

/* Kalman Filter Instances for Sensor Fusion --------------------------------*/
// Vision-based measurement filters
KFPTypeS kfp_yaw_vision = {0.02, 0, 0.01, 0.1, 0};        // Vision yaw angle filter
KFPTypeS kfp_pitch_vision = {0.02, 0, 0.01, 0.1, 0};      // Vision pitch angle filter

// IMU-based measurement filters
KFPTypeS kfp_yaw_imuangle = {0.02, 0, 0.001, 2, 0};  // yaw IMU angle filter
KFPTypeS kfp_yaw_imuspeed = {0.02, 0, 0.001, 1, 0};  // yaw IMU speed filter

// Motor encoder-based measurement filters
KFPTypeS kfp_pitch_angle = {0.02, 0, 0.001, 2, 0};        // Pitch motor angle filter
KFPTypeS kfp_pitch_speed = {0.02, 0, 0.001, 1, 0};        // Pitch motor speed filter
KFPTypeS kfp_yaw_angle = {0.02, 0, 0.001, 2, 0};     // yaw motor angle filter
KFPTypeS kfp_yaw_speed = {0.002, 0, 0.01, 1, 0};     // yaw motor speed filter

/* Gimbal Mechanical Parameters and Limits -----------------------------------*/
static float Yaw_MidPoint_MotorAngle =0.0f;// 1.60914588;    // Lower yaw motor mechanical midpoint (rad)  记得测试更改 机械0角度
static float Pitch_MidPoint_MotorAngle = -0.622802794f;    // Pitch motor mechanical midpoint (rad) 记得测试更改 机械0角度
static float Pitch_LimitMax_Angle = 0.52359886f;            // Maximum pitch angle limit (rad)  记得测试更改，机械限幅pitch
static float Pitch_LimitMin_Angle = -0.20943955f;          // Minimum pitch angle limit (rad)


/**
云台主任务函数
 */
void Control_GIMBAL_Task(void const * argument) 
{
    /* USER CODE BEGIN Control_Task */
    vTaskDelay(1000);  // Initial delay for system stabilization
    TickType_t Control_Task_SysTick = 0;
    Control_Init(&Control_Gimbal_Info);  // Initialize gimbal control system
//		YawSMC(&SMC_control_yaw,580,0,34,0,0.0002,21,27,2,0.0000177,5400); // c  k  ref error_esp q p u_max j epsilon
//			YawSMC(&SMC_control_yaw,72,0,415,0,0.000008,21,27,2,0.0000177,6000); // c  k  ref error_esp q p u_max j epsilon
	YawSMC(&SMC_control_yaw,700,0,860,0,0.000008,21,27,2,0.0000177,1000); // c  k  ref error_esp q p u_max j epsilon  测试速控
			/* Infinite control loop */
    for(;;) 
    {
        Control_Task_SysTick = osKernelSysTick();  // Get current system tick

        // Execute control loop sequence
        Control_Measure_Update(&Control_Gimbal_Info);     // Update sensor measurements with filtering
        Control_Mode_Update(&Control_Gimbal_Info);        // Update control mode based on RC input
        Control_Target_Update(&Control_Gimbal_Info);      // Update target positions and velocities
        Control_Gimbal_Info_Update(&Control_Gimbal_Info); // Calculate and update motor outputs
				USART_Vofa_Justfloat_Transmit(Control_Gimbal_Info.Target.Yaw_Angle,Control_Gimbal_Info.Measure.Yaw_Motor_Angle,0);
        osDelay(1);  // 1ms delay for 1kHz control frequency
    }
}

/**
 * @brief Initialize gimbal control system parameters and controllers
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
static void Control_Init(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
    // Initialize PID controllers with anti-windup configuration
    PID_Init(&Pitch_Angle_PID, PID_ANTI_WINDUP, Pitch_Angle_PID_Param);
    PID_Init(&Pitch_Velocity_PID, PID_ANTI_WINDUP, Pitch_Velocity_Param);

    PID_Init(&Yaw_Angle_PID, PID_ANTI_WINDUP_DEGREE, Yaw_Angle_PID_Param);
    PID_Init(&Yaw_Velocity_PID, PID_ANTI_WINDUP, Yaw_Velocity_PID_Param);

    // Initialize target positions to zero
    Control_Gimbal_Info->Target.Pitch_Angle = 0;
    Control_Gimbal_Info->Target.Yaw_Angle = 0;

    Control_Gimbal_Info->Type = GIMBAL_SLEEP;  // Start in sleep mode
    


    // Initialize motor output values to zero
    Control_Gimbal_Info->SendValue_yaw = 0;
    Control_Gimbal_Info->SendValue_pitch= 0;
    
}

/**
 * @brief Update sensor measurements with Kalman filtering
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
static void Control_Measure_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
    // Update pitch axis measurements with filtering
    Control_Gimbal_Info->Measure.Pitch_Motor_Angle = KalmanFilter(&kfp_pitch_angle, Pich_Motor.Data.Angle / 180.f * PI) - Pitch_MidPoint_MotorAngle;
    Control_Gimbal_Info->Measure.Pitch_Motor_Gyro = KalmanFilter(&kfp_pitch_speed, (float)Pich_Motor.Data.Velocity * 2.0f * PI / 60.0f );

    // Update yaw axis IMU measurements with filtering
    Control_Gimbal_Info->Measure.Yaw_IMU_Angle = INS_Info.Yaw_Angle / 180.f * PI;
    Control_Gimbal_Info->Measure.Yaw_IMU_Gyro = KalmanFilter(&kfp_yaw_imuspeed, INS_Info.Yaw_Gyro);

    // Update lower yaw motor measurements with filtering
    Control_Gimbal_Info->Measure.Yaw_Motor_Gyro = (float)DM_Yaw_Motor.Data.Velocity;//KalmanFilter(&kfp_yaw_speed, (float)DM_Yaw_Motor.Data.Velocity);  // rad/s
    Control_Gimbal_Info->Measure.Yaw_Motor_Angle = (float)DM_Yaw_Motor.Data.Angle - Yaw_MidPoint_MotorAngle;

    // Normalize angles to [-π, π] range
    while (Control_Gimbal_Info->Measure.Yaw_Motor_Angle > PI) Control_Gimbal_Info->Measure.Yaw_Motor_Angle -= 2 * PI;
    while (Control_Gimbal_Info->Measure.Yaw_Motor_Angle < -PI) Control_Gimbal_Info->Measure.Yaw_Motor_Angle += 2 * PI;
    
   
    while (Control_Gimbal_Info->Measure.Yaw_IMU_Angle > PI) Control_Gimbal_Info->Measure.Yaw_IMU_Angle -= 2 * PI;
    while (Control_Gimbal_Info->Measure.Yaw_IMU_Angle < -PI) Control_Gimbal_Info->Measure.Yaw_IMU_Angle += 2 * PI;
	

	
	
}

/**
 * @brief Update target positions based on control mode and input sources
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
static void Control_Target_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
    static Control_Gimbal_Mode_Type_e last_type = GIMBAL_SLEEP;

    if (Control_Gimbal_Info->Type == GIMBAL_SLEEP) 
    {
        // Sleep mode: zero all targets
        Control_Gimbal_Info->Target.Pitch_Angle = 0;
        Control_Gimbal_Info->Target.Yaw_Angle = 0;
    }

    else if ((Control_Gimbal_Info->Type == GIMBAL_REMOTE_JIXIE)||(Control_Gimbal_Info->Type == GIMBAL_REMOTE_IMU)) 
    {
				float pitch_add;
				pitch_add = vtm_rc_data.mouse.y*-0.000050733f;
        // Remote control mode: use RC channel inputs
        Control_Gimbal_Info->Target.Pitch_Angle = remote_ctrl.rc.ch[1] * 0.001f ; 
				Control_Gimbal_Info->Target.Pitch_Angle += pitch_add;
        Control_Gimbal_Info->Target.Yaw_Angle +=remote_ctrl.rc.ch[0] * -0.000005f; //+ vtm_rc_data.mouse.x* -0.000050733f;
//				Control_Gimbal_Info->Target.Yaw_Angle =1.2f;
			
				SMC_control_yaw.target = Control_Gimbal_Info->Target.Yaw_Angle;
    }

    // Normalize lower yaw target angle to [-π, π] range
    while (Control_Gimbal_Info->Target.Yaw_Angle > PI) Control_Gimbal_Info->Target.Yaw_Angle -= 2 * PI;
    while (Control_Gimbal_Info->Target.Yaw_Angle < -PI) Control_Gimbal_Info->Target.Yaw_Angle += 2 * PI;
    
    last_type = Control_Gimbal_Info->Type;  // Record current mode for transition detection
}

/**
 * @brief Update control mode based on RC switch position and connection status
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
static void Control_Mode_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
    static uint8_t last_remote_value = 0x02;

    // Mode selection based on RC switch position
    if (remote_ctrl.rc.s[0] == 0x02) 
    {
        Control_Gimbal_Info->Type = GIMBAL_SLEEP;
    }
    else if (remote_ctrl.rc.s[0] == 0x03) 
    {
        Control_Gimbal_Info->Type = GIMBAL_REMOTE_JIXIE;
    }
    else if (remote_ctrl.rc.s[0] == 0x01) 
    {
        Control_Gimbal_Info->Type = GIMBAL_REMOTE_IMU;
    }
		
		
    last_remote_value = remote_ctrl.rc.s[0];  // Record switch position

    // Execute the appropriate control function based on current mode

}

/**
 * @brief Update output values to be sent to motors
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
static void Control_Gimbal_Info_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
/*
	1		GIMBAL_REMOTE_IMU
	3		GIMBAL_REMOTE_JIXIE
	2   GIMBAL_SLEEP
	*/
		    switch (Control_Gimbal_Info->Type) 
    {
        case GIMBAL_SLEEP:
            // No active control in sleep mode
            break;
        case GIMBAL_REMOTE_JIXIE:
            Control_Gimbal_Remote(Control_Gimbal_Info);
						break;
				case GIMBAL_REMOTE_IMU:
            Control_Gimbal_Remote(Control_Gimbal_Info);
						break;
      
    }
    // Set output values from PID controllers
    Control_Gimbal_Info->SendValue_pitch = (Pitch_Velocity_PID.Output);
    Control_Gimbal_Info->SendValue_yaw = SMC_control_yaw.u;//+ Yaw_Velocity_PID.Output;//(SMC_control_yaw.u);//(Yaw_Velocity_PID.Output);
    
    // Zero outputs in sleep mode for safety
    if (Control_Gimbal_Info->Type == GIMBAL_SLEEP) 
    {
        Control_Gimbal_Info->SendValue_pitch = 0;
        Control_Gimbal_Info->SendValue_yaw= 0;
    }
}

/**
 * @brief Remote control mode implementation
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
void Control_Gimbal_Remote(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
    // Apply pitch angle limits for safety
    VAL_LIMIT(Control_Gimbal_Info->Target.Pitch_Angle, -Pitch_LimitMax_Angle, -Pitch_LimitMin_Angle);
    
    // Pitch axis cascade control: position loop + velocity loop
    PID_Calculate(&Pitch_Angle_PID, Control_Gimbal_Info->Target.Pitch_Angle, Control_Gimbal_Info->Measure.Pitch_Motor_Angle);
    PID_Calculate(&Pitch_Velocity_PID, Pitch_Angle_PID.Output + 18.0f * (0 - Control_Gimbal_Info->Measure.Pitch_Motor_Gyro), 
                  Control_Gimbal_Info->Measure.Pitch_Motor_Gyro);

if (Control_Gimbal_Info->Type == GIMBAL_REMOTE_IMU) 
{
	// Lower yaw axis cascade control (motor encoder based)
    PID_Calculate(&Yaw_Angle_PID, Control_Gimbal_Info->Target.Yaw_Angle, Control_Gimbal_Info->Measure.Yaw_IMU_Angle);
    PID_Calculate(&Yaw_Velocity_PID, Yaw_Angle_PID.Output + 0.100f * (0 - Control_Gimbal_Info->Measure.Yaw_IMU_Gyro), 
                  Control_Gimbal_Info->Measure.Yaw_IMU_Gyro);
}

if (Control_Gimbal_Info->Type == GIMBAL_REMOTE_JIXIE) 
{
//	
	SMC_task(&SMC_control_yaw,Control_Gimbal_Info->Measure.Yaw_Motor_Angle, Control_Gimbal_Info->Measure.Yaw_Motor_Gyro);   
//	
	PID_Calculate(&Yaw_Velocity_PID, SMC_control_yaw.u ,Control_Gimbal_Info->Measure.Yaw_Motor_Gyro);
	
	// Lower yaw axis cascade control (motor encoder based)
//    PID_Calculate(&Yaw_Angle_PID, Control_Gimbal_Info->Target.Yaw_Angle, Control_Gimbal_Info->Measure.Yaw_Motor_Angle);
//    PID_Calculate(&Yaw_Velocity_PID,Yaw_Angle_PID.Output  , 
//                  Control_Gimbal_Info->Measure.Yaw_Motor_Gyro);
	
//		Yaw_Velocity_PID.Output = SMC_control_yaw.u;
//	
}

	
	
}

