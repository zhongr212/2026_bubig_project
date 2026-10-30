#include "Q_math.h"
#include "stdio.h"

//平方根快速导数算法
float Q_sqrt(float number){
	
	long i;
	float x2,y2;
	const float three=1.5f;
	
	x2=number*0.5f;
	y2=number;
	
	i=*(long*)&y2;
	
	i=0x5f375a86 - (i>>1);
	
	y2=*(float*)&i;
	
	y2=y2*(three - (x2*y2*y2));
	y2=y2*(three - (x2*y2*y2));
	
	return number*y2; 
}



