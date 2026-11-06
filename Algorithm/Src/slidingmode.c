#include "slidingmode.h"
#include "math.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "pid.h"

SMC SMC_control_yaw;
SMC SMC_control_pitch;


float error;
float error_last;
float target_dot;
float target_ddot;
float target_last;

float s;
float ds;

float Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}

float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}


/*
c对应期望收敛时间（快速趋近滑模面
k与滑模面趋近时间相关
j 整体放大倍数
epsilon 克服摩擦5
	*/
void YawSMC(SMC* SMC_control,float c1,float c2,float k,float target,float error_eps,float q,float p,float u_max,float J,float epsilon )
{
	SMC_control->C1 = c1;
	SMC_control->C2 = c2;
	SMC_control->K = k;
	SMC_control->target = target;
  SMC_control->error_eps = error_eps;
	SMC_control->q = q;
	SMC_control->p = p;
	SMC_control->u_max = u_max;
	SMC_control->J = J;
	SMC_control->epsilon = epsilon;
	target_dot = 0.0f;
  target_ddot = 0.0f;
  error_last = 0.0f;
  SMC_control->u = 0.0f;

}
	
	
void err_tran(float* error)
{
  while (*error > PI) *error -= 2*PI;
  while (*error < -PI) *error += 2*PI;
}
float err_tt(float a,float b)
{
float erro = a-b;
	while (erro > PI) erro -= 2*PI;
  while (erro < -PI) erro += 2*PI;
	return erro;
}
void SMC_task(SMC* SMC_control,float angle_now,float angle_vel)
{
	
	SMC_control->angle = angle_now;
	SMC_control->ang_vel = angle_vel;
	float e_qp;
	float qp = SMC_control->q/SMC_control->p;

	
	
	error = SMC_control->angle - SMC_control->target;
	err_tran(&error);
	float error_dot = SMC_control->ang_vel - target_dot;
	target_ddot = err_tt(SMC_control->target,target_last) - target_dot;//(SMC_control.target - target_last) - target_dot;
	target_dot = (SMC_control->target - target_last);
	err_tran(&target_dot);
	if (fabs(error) < SMC_control->error_eps)
	{
		SMC_control->u = 0;
		return;
	}

	if (error < 0)
		e_qp = -pow(fabs(error), qp);
	else
		e_qp = pow(fabs(error), qp);

	s = error_dot + SMC_control->C1 * e_qp;
	ds = -SMC_control->epsilon * Sat(s) - SMC_control->K * s;
	SMC_control->u = SMC_control->J * (target_ddot + ds - SMC_control->C2 *error  - SMC_control->C1 * qp * error_dot * e_qp / (error) );

	if (fabs(error) < 1)
	{
		error = SMC_control->angle  - SMC_control->target;
		err_tran(&error);
		s = SMC_control->C1 * error + error_dot;//smc surface
		SMC_control->u = SMC_control->J * (target_ddot - SMC_control->C2 *error  - SMC_control->C1 * error_dot - SMC_control->epsilon * Sat(s) - SMC_control->K * s);
	}

	if (SMC_control->u > SMC_control->u_max)
		SMC_control->u = SMC_control->u_max;
	if (SMC_control->u < -SMC_control->u_max)
		SMC_control->u = -SMC_control->u_max;

	target_last = SMC_control->target;





}
