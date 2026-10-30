#include "PowerContral.h"
#include "RLS.h"


float* controlledOutputs;//接受传入的新的电流值



void Power_init(struct PowerObj* powerObjs,float* out, float* Output,float* Velocity,float* set_vel,int PidMax)
{

		for(int i=0;i<4;i++)
	{
		powerObjs[i].pidout = out[i];
	  powerObjs[i].pidOutput = (int16_t)(Output[i]);
		powerObjs[i].curAv = Velocity[i]*0.0055116 ;//rad/s(公式推的，readme没说)2π/60 / 19
		powerObjs[i].setAv = (set_vel[i])/Wheel_radius;//除轮半径得角速度
		powerObjs[i].pidMaxOutput = PidMax;
	}
}//获得必要计算数据

void energy_init(struct Manager* manager,float baseBuffSet,float fullBuffSet,float refereeMaxPower)
{

	manager->userConfiguredMaxPower = 180.0f;//设定初始最大公率
	manager->baseBuffSet = baseBuffSet;//超电满能量*0.1(初始)
  manager->fullBuffSet = fullBuffSet;//超电满能量*0.9（初始）

	manager->refereeMaxPower = refereeMaxPower;//裁判系统允许最大功率
}//获得电容参数

	
void x_param_init(struct PowerObj* powerObjs)
{
	
	static float rls_sample_vector[2];
	static float rls_sample_out;
    // 参数验证
    if(powerObjs == NULL)
        return;
		
    
    float k0 = torque*conversion;//扭矩电流比（转矩系数*减速比*数字模拟转换系数）
		rls_sample_vector[0] =(fabs(powerObjs[0].curAv) + fabs(powerObjs[1].curAv) + fabs(powerObjs[2].curAv) + fabs(powerObjs[3].curAv));
		rls_sample_vector[1] =(powerObjs[0].pidOutput)*(powerObjs[0].pidOutput)*k0*k0;
		rls_sample_vector[1] +=(powerObjs[1].pidOutput)*(powerObjs[1].pidOutput)*k0*k0;
		rls_sample_vector[1] +=(powerObjs[2].pidOutput)*(powerObjs[2].pidOutput)*k0*k0;
		rls_sample_vector[1] +=(powerObjs[3].pidOutput)*(powerObjs[3].pidOutput)*k0*k0;
    rls_sample_out = -Super_C_Msg.chassisPower - 4.866f - (powerObjs[0].curAv)*(powerObjs[0].pidOutput)*k0- (powerObjs[1].curAv)*(powerObjs[1].pidOutput)*k0- (powerObjs[2].curAv)*(powerObjs[2].pidOutput)*k0- (powerObjs[3].curAv)*(powerObjs[3].pidOutput)*k0;
    RLS_task(&manager, rls_sample_vector, rls_sample_out);

}//RLS参数运算输入

float* Power_task(struct PowerObj* powerObjs,float* out , float* Output,float* Velocity,float* set_vel,int PidMax)
{
		Power_init(powerObjs,out,Output,Velocity,set_vel,PidMax); 
		x_param_init(powerObjs);
		controlledOutputs = Power_getControlledOutput(powerObjs);
		return controlledOutputs;
}//计算函数

