#ifndef CONTROL_GIMBAL_TASK_H
#define CONTROL_GIMBAL_TASK_H
#include "cmsis_os.h"

typedef struct{
float kp;
float ki;
float kd;
float Deadband;
float limitIntegral;
float limitOutput;




}pid_yawprogram_Info_Typedef;


typedef struct{
	float angle[2];
	float search_angle[2];
	float heat[4];
	float yaw_zero_angle[4];
}test_program_Info_Typedef;
extern test_program_Info_Typedef test_program_Info; 
extern pid_yawprogram_Info_Typedef pid_yawprogram_Info;
extern void fire_choose();
#endif