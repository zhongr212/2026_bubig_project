/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "DM_imu.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "bsp_can.h"
#include "config.h"
#include "arm_math.h"
#include "Q_math.h"
#include "RLS.h"
#include "Referee_System.h"
#include "Control_Gaimbal_task.h"
#include "Power.h"

#define chassis_spin_speed 5        //小陀螺速度（未测试）
#define Transform_Angle 0.0439453125 // 转化为角度值
#define R 0.135
#define sin45 0.70710678118654752440084436210485
float Chassis_RC_Max_Speed = 5.f;      // Maximum speed from RC input (m/s)
float Set_x,Set_y;                     //设定的速度	
float w0, w1, w2, w3;                  // Individual wheel velocities
float aver_vx = 0.0f, aver_vy = 0.0f;  // Average chassis velocities in body frame
float world_vx, world_vy;              // Velocities in world coordinate frame

static void Control_Init(Chassis_Info_Typedef *Chassis_Info);
static void Control_Mode(Chassis_Info_Typedef *Chassis_Info);
static void Chassis_Cal(Chassis_Info_Typedef *Chassis_Info);
static void Control_Measure_Update(Chassis_Info_Typedef *Chassis_Info);
static void Control_Target_Update(Chassis_Info_Typedef *Chassis_Info);
static void Control_Chassis_Info_Update(Chassis_Info_Typedef *Chassis_Info);
	
// 变量声明
LowPassFilter1p_Info_TypeDef LPF_ChassisPower;
LowPassFilter1p_Info_TypeDef LPF_Motor[4];
Chassis_Info_Typedef Chassis_Info;

PID_Info_TypeDef Target_Velocity_PID[2];
PID_Info_TypeDef Wheel_Velocity_PID[4];  // Four wheel velocity controllers
PID_Info_TypeDef Chassis_Follow_PID;     // Chassis follow gimbal angle controller
PID_Info_TypeDef Target_Velocity_PID[2];


static float Wheel_0Velocity_PID_Param[7] = {1800.0f, 0.0f, 5.0f, 0.1f, 0.1f, 50.f, 20000.f};
static float Wheel_1Velocity_PID_Param[7] = {1800.0f, 0.0f, 5.0f, 0.1f, 0.1f, 50.f, 20000.f};
static float Wheel_2Velocity_PID_Param[7] = {1800.0f, 0.0f, 5.0f, 0.1f, 0.1f, 50.f, 20000.f};
static float Wheel_3Velocity_PID_Param[7] = {1800.0f, 0.0f, 5.0f, 0.1f, 0.1f, 50.f, 20000.f};
static float Target_xVelocity_PID_Param[7] = {2.0f, 0.000000f, 0.001f, 0.0f, 0.1f, 0.1f, 10.0f};
static float Target_yVelocity_PID_Param[7] = {2.0f, 0.000000f, 0.001f, 0.0f, 0.1f, 0.1f, 10.0f};
// Chassis follow gimbal control parameters
static float Chassis_Follow_PID_Param[7] = {20.f, 0.0f, 10.0f, 0.f, 0.f, 0.f, 1200.f};

TickType_t Control_Task_SysTick = 0;

//超电专区
void energy_init(struct Manager* manager);//能量环的计算函数
void x_param_init(struct PowerObj powerObjs[4]);//RLS的计算函数
extern Super_C_Msg_t Super_C_Msg;//超电结构体
extern struct Manager manager;//功率控制总结构体
extern struct PowerStatus powerStatus;//功率状态结构体
extern struct PowerObj powerObjs[4]; //传入的参数，轮子角速度等

float Set_x,Set_y;//设定的速度	
float Set[4];//每个轮的速度
static float* controlledOutputs;//接受传入的新的电流值
float K_energy[2];//能量环控制值
void Power_init(void);//传入参数
static float rls_sample_vector[2];

void Control_Task(void const *argument)
{
	/* USER CODE BEGIN Control_Task */

	Control_Init(&Chassis_Info);
	Manager_init(&manager,SENTRY,RLS_ENABLE,1,1,3.5);//初始化功率计算的参数
	energy_init(&manager);//初始能量环相关参数
	K_energy[0]=10.0;
	K_energy[1]=0.000001;
	manager.userConfiguredMaxPower = 180.0f;//设定初始最大公率
	/* Infinite loop */
	for (;;)
	{

		energy_control(&manager,K_energy[0],K_energy[1]);//能量环控制（得到功率阈值）
		Control_Task_SysTick = osKernelSysTick();
		Control_Mode(&Chassis_Info);
		Chassis_Cal(&Chassis_Info);
		Control_Measure_Update(&Chassis_Info);
		Control_Target_Update(&Chassis_Info);
		Control_Chassis_Info_Update(&Chassis_Info);

		//USART_Vofa_Justfloat_Transmit(MiniPc_Receive_Auto.Vision_Yaw,YAW_Motor.Data.Angle,0); // 测试用
		osDelay(1);
	}
}
/* USER CODE END Control_Task */
static void Control_Init(Chassis_Info_Typedef *Chassis_Info){
		LowPassFilter1p_Init(&LPF_ChassisPower,0.7);
	
		PID_Init(&Wheel_Velocity_PID[0], PID_VELOCITY, Wheel_0Velocity_PID_Param);
    PID_Init(&Wheel_Velocity_PID[1], PID_VELOCITY, Wheel_1Velocity_PID_Param);
    PID_Init(&Wheel_Velocity_PID[2], PID_VELOCITY, Wheel_2Velocity_PID_Param);
    PID_Init(&Wheel_Velocity_PID[3], PID_VELOCITY, Wheel_3Velocity_PID_Param);
    PID_Init(&Target_Velocity_PID[0], PID_POSITION, Target_xVelocity_PID_Param);
	  PID_Init(&Target_Velocity_PID[1], PID_POSITION, Target_yVelocity_PID_Param);
    // Initialize chassis follow gimbal PID controller
    PID_Init(&Chassis_Follow_PID, PID_ANGLE, Chassis_Follow_PID_Param);
    
    // Initialize motor output values to zero
    Chassis_Info->SendValue[0] = 0;
    Chassis_Info->SendValue[1] = 0;
    Chassis_Info->SendValue[2] = 0;
    Chassis_Info->SendValue[3] = 0;


    // Set pointer to gimbal angle for chassis following
    Chassis_Info->Chassis_Follow_Gimbal_Angle_TM = &(Control_Gimbal_Info.Measure.Yaw_Motor_Angle);
	
}

static void Control_Mode(Chassis_Info_Typedef *Chassis_Info){
	// 模式选择   1,3,2
		switch (remote_ctrl.rc.s[1]){
		case 1: // 第一位为1的情况
			{Chassis_Info->Type = chassis_follow_gimbal;}
			break;
		case 3: // 第一位为2的情况
			{Chassis_Info->Type = chassis_mode;}
			break;

		case 2: // 第一位为3的情况
			{Chassis_Info->Type = sleep;}
			break;
		}
	}

static void Chassis_Cal(Chassis_Info_Typedef *Chassis_Info){
       // Read individual wheel velocities
        w0 = Motor_chassis[0].Data.Velocity;
        w1 = Motor_chassis[1].Data.Velocity;
        w2 = Motor_chassis[2].Data.Velocity;
        w3 = Motor_chassis[3].Data.Velocity;
        
        // Calculate average velocities in body frame (Mecanum wheel kinematics)
        aver_vx = (-w0 - w1 + w2 + w3) * 0.00798f * 1.414f / 4.0f / 8;
        aver_vy = (-w0 + w1 + w2 - w3) * 0.00798f * 1.414f / 4.0f / 8;
        
			
        // Transform body frame velocities to world frame using gimbal yaw angle
        float delta_angle = 0;//*Gimbal_Yaw_Angle_Chassis; //- Control_Gimbal_Info.Measure.Lower_Yaw_IMU_Angle;
        world_vx = -aver_vx * arm_cos_f32(delta_angle) + aver_vy * arm_sin_f32(delta_angle);
        world_vy = aver_vx * arm_sin_f32(delta_angle) + aver_vy * arm_cos_f32(delta_angle);
	
}
static void Control_Measure_Update(Chassis_Info_Typedef *Chassis_Info){

	 // Convert motor RPM to linear velocity (m/s)
    // Conversion factor: /60 * 2 * π * (wheel_radius) / reduction_ratio
    const float RPM_TO_MPS = 0.0014104;  // Conversion factor
    
    Chassis_Info->Measure.Wheel_Velocity[0] = Motor_chassis[0].Data.Velocity * RPM_TO_MPS;
    Chassis_Info->Measure.Wheel_Velocity[1] = Motor_chassis[1].Data.Velocity * RPM_TO_MPS;
    Chassis_Info->Measure.Wheel_Velocity[2] = Motor_chassis[2].Data.Velocity * RPM_TO_MPS;
    Chassis_Info->Measure.Wheel_Velocity[3] = Motor_chassis[3].Data.Velocity * RPM_TO_MPS;
    
    // Calculate average chassis velocity (for monitoring)
    Chassis_Info->Measure.velTheta = (Chassis_Info->Measure.Wheel_Velocity[0] + 
                                             Chassis_Info->Measure.Wheel_Velocity[1] + 
                                             Chassis_Info->Measure.Wheel_Velocity[2] + 
                                             Chassis_Info->Measure.Wheel_Velocity[3]) / 4.0f;
	
	//	for (i = 0; i < 4; i++){
//		Chassis_Info->Measure.Chassis_Velocity[i] = Motor_Advance[i].Data.Velocity;
//		Chassis_Info->Measure.Chassis_Angle[i] = Motor_Course[i].Data.Angle;
//		Chassis_Info->Measure.Chassis_Angle_v[i] = Motor_Course[i].Data.Velocity;
//	}
//	Control_Info.Measure.Gimbal_Yaw_Angle = Gimbal.Yaw_Angle;
//	if(Gimbal.Yaw_Angle>=180)  Gimbal.Yaw_Angle-=360;   
//  else if(Gimbal.Yaw_Angle<=-180) Gimbal.Yaw_Angle+=360;
//	
//	Control_Info.Measure.Gimbal_Yaw_Gyro = Gimbal.Yaw_Gyro;
//	
//		Chassis_Info->trigger.last_state[0] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
//		Chassis_Info->trigger.last_state[1] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
//		Chassis_Info->trigger.last_state[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
//		Chassis_Info->trigger.last_state[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}

static void Control_Target_Update(Chassis_Info_Typedef *Chassis_Info){      //以头为正方向
	    // Remote control modes (normal and spin)
//    if (Chassis_Info->Type == CHASSIS_REMOTE || Chassis_Info->Type == CHASSIS_REMOTE_SPIN) {
        // Convert RC channel values to target velocities
			float Target_velX,Target_velY;
      float Set_velX = (float)(remote_ctrl.rc.ch[2]) / 660.0f*Chassis_RC_Max_Speed;
      float Set_velY = (float)(remote_ctrl.rc.ch[3]) / 660.0f*Chassis_RC_Max_Speed;
			Set_x = Set_velX;
			Set_y = Set_velY;
			if (Chassis_Info->Type == chassis_follow_gimbal) {
            // Normal mode: maintain chassis orientation relative to gimbal
            Chassis_Info->Target.velTheta = PID_Calculate(&Chassis_Follow_PID, 0, 
                                                                  -*Chassis_Info->Chassis_Follow_Gimbal_Angle_TM);
        }
        else if (Chassis_Info->Type == chassis_mode) {
            // Spin mode: apply constant rotation for evasion
   					Chassis_Info->Target.velTheta = (float)(remote_ctrl.rc.ch[4]) / 660.0f*Chassis_RC_Max_Speed;;
					// Reduce spin intensity during high-speed movement
           
        }
			if(Chassis_Info->Type == chassis_spin)
			{
			Target_velX=Set_velX ;
			Target_velY=Set_velY ;
			}
			else if(Chassis_Info->Type == 0)
			{
			Target_velX = PID_Calculate(&Target_Velocity_PID[0], Set_velX,world_vx);
			Target_velY = PID_Calculate(&Target_Velocity_PID[1], Set_velY,world_vy);	
			}

        // Calculate rotation matrix components for chassis-gimbal coordination
        float Chassis_Gimbal_sin = arm_sin_f32(-*Chassis_Info->Chassis_Follow_Gimbal_Angle_TM);
        float Chassis_Gimbal_cos = arm_cos_f32(-*Chassis_Info->Chassis_Follow_Gimbal_Angle_TM);
        
        // Transform input velocities to chassis coordinate frame
        Chassis_Info->Target.velX = Chassis_Gimbal_cos * Target_velX + Chassis_Gimbal_sin * Target_velY;
        Chassis_Info->Target.velY = -Chassis_Gimbal_sin * Target_velX + Chassis_Gimbal_cos * Target_velY;
        
        // Set rotation behavior based on mode
     if ((fabs(Set_velX) > (0.3*Chassis_RC_Max_Speed) || 
          fabs(Set_velY) > (0.3*Chassis_RC_Max_Speed))&&
					(Chassis_Info->Type == chassis_spin)) {
                Chassis_Info->Target.velTheta = 0;
            }  

        

    }


static void Control_Chassis_Info_Update(Chassis_Info_Typedef *Chassis_Info)
{
    // Mecanum wheel inverse kinematics: convert chassis velocities to individual wheel velocities
    Chassis_Info->Target.Wheel_Velocity[0] = Chassis_Info->Target.velX - 
                                                     Chassis_Info->Target.velY - 
                                                     Chassis_Info->Target.velTheta;
    Chassis_Info->Target.Wheel_Velocity[1] = Chassis_Info->Target.velX + 
                                                     Chassis_Info->Target.velY - 
                                                     Chassis_Info->Target.velTheta;
    Chassis_Info->Target.Wheel_Velocity[2] = -Chassis_Info->Target.velX + 
                                                     Chassis_Info->Target.velY - 
                                                     Chassis_Info->Target.velTheta;
    Chassis_Info->Target.Wheel_Velocity[3] = -Chassis_Info->Target.velX - 
                                                     Chassis_Info->Target.velY - 
                                                     Chassis_Info->Target.velTheta;
		
    // Calculate PID outputs for each wheel
    PID_Calculate(&Wheel_Velocity_PID[0], Chassis_Info->Target.Wheel_Velocity[0], 
                  Chassis_Info->Measure.Wheel_Velocity[0]);
    PID_Calculate(&Wheel_Velocity_PID[1], Chassis_Info->Target.Wheel_Velocity[1], 
                  Chassis_Info->Measure.Wheel_Velocity[1]);
    PID_Calculate(&Wheel_Velocity_PID[2], Chassis_Info->Target.Wheel_Velocity[2], 
                  Chassis_Info->Measure.Wheel_Velocity[2]);
    PID_Calculate(&Wheel_Velocity_PID[3], Chassis_Info->Target.Wheel_Velocity[3], 
                  Chassis_Info->Measure.Wheel_Velocity[3]);
		
		Power_init(); //
		x_param_init(powerObjs);
		controlledOutputs = Power_getControlledOutput(powerObjs);
    
    Chassis_Info->SendValue[0] = (int16_t)(Wheel_Velocity_PID[0].Output);
		Chassis_Info->SendValue[1] = (int16_t)(Wheel_Velocity_PID[1].Output);
		Chassis_Info->SendValue[2] = (int16_t)(Wheel_Velocity_PID[2].Output);
		Chassis_Info->SendValue[3] = (int16_t)(Wheel_Velocity_PID[3].Output);
    // Safety: zero outputs in sleep mode
    if (Chassis_Info->Type == sleep) {
        Chassis_Info->SendValue[0] = 0;
        Chassis_Info->SendValue[1] = 0;
        Chassis_Info->SendValue[2] = 0;
        Chassis_Info->SendValue[3] = 0;
    }
}


void Power_init(void)
{
		float vel = Chassis_Info.Target.velTheta;
		Set[0] = Set_x - Set_y - vel;
    Set[1] = Set_x + Set_y - vel;
    Set[2] = -Set_x + Set_y - vel;
    Set[3] = -Set_x - Set_y - vel;
	
		for(int i=0;i<4;i++)
	{
	  powerObjs[i].pidOutput = (int16_t)(Wheel_Velocity_PID[i].Output);
		powerObjs[i].curAv = Motor_chassis[i].Data.Velocity*0.0198647887323;
		powerObjs[i].setAv = (Set[i])/0.071f;//除轮半径得角速度
		powerObjs[i].pidMaxOutput = 20000;
	}
}

void energy_init(struct Manager* manager)
{
	manager->baseBuffSet = 229.5;//超电满能量*0.9
  manager->fullBuffSet = 25.5f;//超电满能量*0.1
	manager->powerBuff = Super_C_Msg.capEnergy;//超电能量
	manager->refereeMaxPower = 75.0f;//裁判系统允许最大功率
}

	
	void x_param_init(struct PowerObj* powerObjs)
{
    // 参数验证
    if(powerObjs == NULL || &manager == NULL)
        return;
    
    float k0 = 0.01562*0.001220703125;//扭矩电流比（转矩系数*减速比*数字模拟转换系数）
		rls_sample_vector[0] +=((powerObjs[0].curAv) + (powerObjs[1].curAv) + (powerObjs[2].curAv) + (powerObjs[3].curAv));
		rls_sample_vector[1] +=(powerObjs[0].pidOutput)*(powerObjs[0].pidOutput)*k0*k0;
		rls_sample_vector[1] +=(powerObjs[1].pidOutput)*(powerObjs[1].pidOutput)*k0*k0;
		rls_sample_vector[1] +=(powerObjs[2].pidOutput)*(powerObjs[2].pidOutput)*k0*k0;
		rls_sample_vector[1] +=(powerObjs[3].pidOutput)*(powerObjs[3].pidOutput)*k0*k0;
   
    RLS_task(&manager, rls_sample_vector, -Super_C_Msg.chassisPower);

    
}



