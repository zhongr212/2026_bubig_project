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
//#include "aiming.h"

/* Internal Control Function Prototypes --------------------------------------*/
static void Control_Init(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Measure_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Target_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Mode_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
static void Control_Gimbal_Info_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);

/* Gimbal Control Mode Function Prototypes -----------------------------------*/
void Control_Gimbal_Remote(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);
void Control_Gimbal_Upper(Control_Gimbal_Info_Typedef *Control_Gimbal_Info);

/* Global Variables ----------------------------------------------------------*/
Control_Gimbal_Info_Typedef Control_Gimbal_Info;  // Main gimbal control information structure

/* PID Controller Parameters -------------------------------------------------*/
// Pitch axis control parameters [KP, KI, KD, Alpha, Deadband, I_MAX, Output_MAX, ...]
static float Pitch_Angle_PID_Param[10] = {6.5f, 0.007f, 18.0f, 0.0f, 0.0f, 5.f, 7.5f, 0.0272f, 0.4363f, 0.0f};
static float Pitch_Velocity_Param[10] = {11000.0f, 48.0f, 3500.0f, 0.0f, 0.0f, 20000.0f, 25000.f, 2.5f, 0.25f, 0};


// Lower yaw axis control parameters
static float Yaw_Angle_PID_Param[10] = {5.0f, 0.01f, 10.0f, 0.0f, 0.0f, 3.f, 3.0f, 0.0972f, 0.4363f, 0.0f};
static float Yaw_Velocity_PID_Param[10] = {11000.0f, 20.0f, 3500.0f, 0.0f, 0.0f, 20000.0f, 25000.f, 2.5f, 0.25f, 0};

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
KFPTypeS kfp_yaw_speed = {0.02, 0, 0.001, 1, 0};     // yaw motor speed filter

/* Gimbal Mechanical Parameters and Limits -----------------------------------*/
static float Yaw_MidPoint_MotorAngle = 1.60914588;    // Lower yaw motor mechanical midpoint (rad)  记得测试更改
static float Pitch_MidPoint_MotorAngle = -0.622802794f;    // Pitch motor mechanical midpoint (rad)
static float Pitch_LimitMax_Angle = 0.52359886f;            // Maximum pitch angle limit (rad)
static float Pitch_LimitMin_Angle = -0.20943955f;          // Minimum pitch angle limit (rad)
//static float Yaw_MidPoint_IMUAngle;                   // Lower yaw IMU reference angle (rad)

/**
 * @brief Main gimbal control task function
 * @param argument RTOS task argument (unused)
 * @retval None
 */
void Control_GIMBAL_Task(void const * argument) 
{
    /* USER CODE BEGIN Control_Task */
    vTaskDelay(1000);  // Initial delay for system stabilization
    TickType_t Control_Task_SysTick = 0;
    Control_Init(&Control_Gimbal_Info);  // Initialize gimbal control system

    /* Infinite control loop */
    for(;;) 
    {
        Control_Task_SysTick = osKernelSysTick();  // Get current system tick

        // Execute control loop sequence
        Control_Measure_Update(&Control_Gimbal_Info);     // Update sensor measurements with filtering
        Control_Mode_Update(&Control_Gimbal_Info);        // Update control mode based on RC input
        Control_Target_Update(&Control_Gimbal_Info);      // Update target positions and velocities
        Control_Gimbal_Info_Update(&Control_Gimbal_Info); // Calculate and update motor outputs

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
    Control_Gimbal_Info->SendValue[0] = 0;
    Control_Gimbal_Info->SendValue[1] = 0;
    Control_Gimbal_Info->SendValue[2] = 0;
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
    Control_Gimbal_Info->Measure.Pitch_Motor_Gyro = KalmanFilter(&kfp_pitch_speed, (float)Pich_Motor.Data.Velocity / 60.0f * 2.0f * PI);

    // Update yaw axis IMU measurements with filtering
    Control_Gimbal_Info->Measure.Yaw_IMU_Angle = DM_IMU_Info.yaw / 180.f * PI;
    Control_Gimbal_Info->Measure.Yaw_IMU_Gyro = KalmanFilter(&kfp_yaw_imuspeed, DM_IMU_Info.gyro[2]);

    // Update lower yaw motor measurements with filtering
    Control_Gimbal_Info->Measure.Yaw_Motor_Gyro = KalmanFilter(&kfp_yaw_speed, (float)DM_Yaw_Motor.Data.Velocity / (2.0f * PI * 60.0f));  // rad/s
    Control_Gimbal_Info->Measure.Yaw_Motor_Angle = (float)DM_Yaw_Motor.Data.Angle / 180.f * PI - Yaw_MidPoint_MotorAngle;

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

    else if (Control_Gimbal_Info->Type == GIMBAL_REMOTE) 
    {
        // Remote control mode: use RC channel inputs
        Control_Gimbal_Info->Target.Pitch_Angle = remote_ctrl.rc.ch[1] * 0.001f;
        Control_Gimbal_Info->Target.Yaw_Angle += remote_ctrl.rc.ch[0] * -0.000005f;
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
        Control_Gimbal_Info->Type = GIMBAL_REMOTE;
    }
    
    last_remote_value = remote_ctrl.rc.s[0];  // Record switch position

    // Execute the appropriate control function based on current mode
    switch (Control_Gimbal_Info->Type) 
    {
        case GIMBAL_SLEEP:
            // No active control in sleep mode
            break;
        case GIMBAL_REMOTE:
            Control_Gimbal_Remote(Control_Gimbal_Info);
						break;
			
      
    }
}

/**
 * @brief Update output values to be sent to motors
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
static void Control_Gimbal_Info_Update(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
{
    // Set output values from PID controllers
    Control_Gimbal_Info->SendValue[0] = (int16_t)(Pitch_Velocity_PID.Output);
    Control_Gimbal_Info->SendValue[1] = (int16_t)(Yaw_Velocity_PID.Output);
    
    // Zero outputs in sleep mode for safety
    if (Control_Gimbal_Info->Type == GIMBAL_SLEEP) 
    {
        Control_Gimbal_Info->SendValue[0] = 0;
        Control_Gimbal_Info->SendValue[1] = 0;
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

    // Lower yaw axis cascade control (motor encoder based)
    PID_Calculate(&Yaw_Angle_PID, Control_Gimbal_Info->Target.Yaw_Angle, Control_Gimbal_Info->Measure.Yaw_IMU_Angle);
    PID_Calculate(&Yaw_Velocity_PID, Yaw_Angle_PID.Output + 0.100f * (0 - Control_Gimbal_Info->Measure.Yaw_IMU_Gyro), 
                  Control_Gimbal_Info->Measure.Yaw_IMU_Gyro);
}


/**
 * @brief Upper computer control mode implementation (vision-based)
 * @param Control_Gimbal_Info Pointer to gimbal control information structure
 * @retval None
 */
//void Control_Gimbal_Upper(Control_Gimbal_Info_Typedef *Control_Gimbal_Info) 
//{
//    // Apply safety limits to target angles
//    VAL_LIMIT(Control_Gimbal_Info->Target.Pitch_Angle, -Pitch_LimitMax_Angle, -Pitch_LimitMin_Angle);
//    VAL_LIMIT(Control_Gimbal_Info->Target.Upper_Yaw_Angle, -UpperYaw_LimitMax_Angle, -UpperYaw_LimitMin_Angle);
//    
//    // Pitch axis cascade control
//    PID_Calculate(&Pitch_Angle_PID, Control_Gimbal_Info->Target.Pitch_Angle, Control_Gimbal_Info->Measure.Pitch_Motor_Angle);
//    PID_Calculate(&Pitch_Velocity_PID, Pitch_Angle_PID.Output + 11.0f * (0 - Control_Gimbal_Info->Measure.Pitch_Motor_Gyro), 
//                  Control_Gimbal_Info->Measure.Pitch_Motor_Gyro);
//    
//    // Upper yaw axis cascade control
//    PID_Calculate(&UpperYaw_Angle_PID, Control_Gimbal_Info->Target.Upper_Yaw_Angle, Control_Gimbal_Info->Measure.UpperYaw_Motor_Angle);
//    PID_Calculate(&UpperYaw_Velocity_PID, UpperYaw_Angle_PID.Output + 18.1f * (0 - Control_Gimbal_Info->Measure.Upper_Yaw_Motor_Gyro), 
//                  Control_Gimbal_Info->Measure.Upper_Yaw_Motor_Gyro);

//    // Lower yaw axis cascade control (IMU based for better absolute positioning)
//    PID_Calculate(&LowerYaw_Angle_PID, Control_Gimbal_Info->Target.Lower_Yaw_Angle, Control_Gimbal_Info->Measure.Lower_Yaw_IMU_Angle);
//    PID_Calculate(&LowerYaw_Velocity_PID, LowerYaw_Angle_PID.Output + 0.100f * (0 - Control_Gimbal_Info->Measure.Lower_Yaw_IMU_Gyro), 
//                  Control_Gimbal_Info->Measure.Lower_Yaw_IMU_Gyro);
//}

