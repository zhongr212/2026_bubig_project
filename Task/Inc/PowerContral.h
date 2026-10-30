#ifndef POWERCONTRAL_H
#define POWERCONTRAL_H


#include "Power.h"
#include "bsp_can.h"

//************************不同电机记得改参数*****************************************//
#define Wheel_radius  0.071f
#define MYPI  3.1415926535897932f
#define reduction_ratio  19.203208556149f
#define conversion 0.001220703125f
#define torque  0.01562f
 


void Power_init(struct PowerObj* powerObjs,float* out, float* dianliu,float* Velocity,float* set_vel,int PidMax);//传入参数
void energy_init(struct Manager* manager,float baseBuffSet,float fullBuffSet,float refereeMaxPower);//能量环的计算函数
void x_param_init(struct PowerObj* powerObjs);//RLS的计算函数
float* Power_task(struct PowerObj* powerObjs,float* out, float* Output,float* Velocity,float* set_vel,int PidMax);


#endif


