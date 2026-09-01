/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "DM_IMU.h"
#include "stdbool.h"
#include "main.h"


#define DMIMU_CC        0xCC
#define DMIMU_READ      0x00
#define DMIMU_WRITE     0x01
#define pi 3.1415926535897

static uint16_t dmimu_can_id = 0x100;  // 根据实际模块修改
static uint16_t dmimu_mst_id = 0x101;  // 根据实际模块修改


DM_IMU_Info_Typedef DM_IMU_Info = {
		.CANFrame = {
				  .TxIdentifier = 0x6FF,
				  .RxIdentifier = 0x11,
			}	
};


/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void IMU_RequestData(FDCAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg)
{
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
	uint32_t returnBox;
	tx_header.DataLength=8;
	tx_header.Identifier=FDCAN_STANDARD_ID;
	tx_header.TxFrameType=FDCAN_DATA_FRAME;
	tx_header.Identifier=0x6FF;
	
	if(HAL_FDCAN_GetTxFifoFreeLevel(hcan)>1)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&tx_header,cmd);
	}
}


void DM_IMU_Accel_Update(uint8_t* pData)//加速度
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	DM_IMU_Info.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	DM_IMU_Info.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	DM_IMU_Info.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void DM_IMU_Gyro_Update(uint8_t* pData)//角速度
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	DM_IMU_Info.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	DM_IMU_Info.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	DM_IMU_Info.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void DM_IMU_Euler_Update(uint8_t* pData)//欧拉角(度)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	DM_IMU_Info.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16)/180*pi;
	DM_IMU_Info.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16)/180*pi;
	DM_IMU_Info.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16)/180*pi;
}
 
void DM_IMU_Quaternion_Update(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	DM_IMU_Info.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	DM_IMU_Info.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	DM_IMU_Info.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	DM_IMU_Info.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void DM_IMU_Info_Update(uint8_t* pData)
{
	switch(pData[0])
	{
		case 1:
			DM_IMU_Accel_Update(pData);
			break;
		case 2:
			DM_IMU_Gyro_Update(pData);
			break;
		case 3:
			DM_IMU_Euler_Update(pData);
			break;
		case 4:
			DM_IMU_Quaternion_Update(pData);
			break;
	}
}
