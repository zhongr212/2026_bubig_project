#ifndef SLIDINGMODE_H
#define SLIDINGMODE_H

typedef struct
{
	float C1;
	float C2;
	float K;
	float target;
	float error_eps;
	float q;
	float p;
	float u_max;
	float J;
	float angle;
	float ang_vel;
	float epsilon;

	float u;

}SMC;

extern void YawSMC(SMC* SMC_control,float c1,float c2,float k,float target,float error_eps,float q,float p,float u_max,float J,float epsilon );
extern void SMC_task(SMC* SMC_control,float angle_now,float angle_vel);
extern SMC SMC_control_yaw;
extern SMC SMC_control_pitch;

















#endif
