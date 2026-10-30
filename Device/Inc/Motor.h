#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief typedef enum that contains the Frame Identifier for DJI Motor Device.
 */
typedef enum
{
  DJI_TxFrame_HIGH = 0x1feU,
  DJI_TxFrame_LOW = 0x200U,
  DJI_RxFrame_MIDDLE = 0x204U,
  DJI_MotorFrameId_NUM,
}DJI_MotorFrameId_e;

/**
 * @brief typedef enum that contains the Error status for Motor Device.
 */
typedef enum
{
  MOTOR_ERROR_NONE = 0x00U,   /*!< no error */
  MOTOR_CAN_OFFLINE = 0x01U,    /*!< CAN transfer failed */
  MOTOR_OVER_TEMPERATURE = 0x02U,   /*!< abnormal motor temperature */
}Motor_Status_e;

/**
 * @brief typedef enum that contains the type of RMD Motor Device.
 */
typedef enum{
	  RMD_L9025,
    RMD_MOTOR_TYPE_NUM,
}RMD_Motor_Type_e;

/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
	  DJI_AGV3508,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;


/**
 * @brief typedef structure that contains the information for the Motor Error handler.
 */
typedef struct 
{
  uint16_t ErrorCount;    /*!< Error status judgment count */
  Motor_Status_e Status;   /*!< Error status */
  uint16_t Temp_ErrorCount;    /*!< Error status judgment count */
  uint16_t CAN_ErrorCount; 
  Motor_Status_e Last_status;
}Motor_ErrorrHandler_Typedef;

/**
 * @brief typedef structure that contains the information for the Motor CAN Transfer.
 */
typedef struct
{
  uint32_t TxStdId;   /*!< Specifies CAN transfer identifier */
  uint32_t RxStdId;   /*!< Specifies CAN transfer identifier */
  uint8_t FrameIndex;   /* index for motor transmit frame */
}Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the General information for the Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */
  int16_t Last_Current;
  int16_t  Current;   /*!< Motor electric current */
  int16_t  Velocity;    /*!< Motor rotate velocity */
  int16_t  Encoder;   /*!< Motor encoder angle */
  int16_t  Last_Encoder;   /*!< previous Motor encoder angle */
  float    Angle;   /*!< Motor angle in degree */
  uint8_t  Temperature;   /*!< Motor Temperature */
}Motor_GeneralInfo_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef CANFrame;    /*!< information for the CAN Transfer */
	Motor_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< information for the Motor Error */

}DJI_Motor_Info_Typedef;
/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
	uint8_t order;   /*!< Motor feedback order */
	RMD_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef CANFrame;    /*!< information for the CAN Transfer */
	Motor_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< information for the Motor Error */
}RMD_L9025_Info_Typedef;

/**
 * @brief  typedef enum that control mode the type of DM_Motor.
 */
typedef enum
{
  MIT,
	POSITION_VELOCITY,
	VELOCITY,
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
typedef struct 
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}DM_Motor_Param_Range_Typedef;


/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
typedef struct 
{
	
  bool Initlized;    /*!< init flag */
  uint8_t  State; 	 /*!< Motor Message */
  uint16_t  P_int;   /*!< Motor Positon  uint16 */
	uint16_t  V_int;   /*!< Motor Velocity uint16 */
	uint16_t  T_int;   /*!< Motor Torque   uint16 */
	float  Position;   /*!< Motor Positon  */
  float  Velocity;   /*!< Motor Velocity */
  float  Torque;     /*!< Motor Torque   */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS   */
	float  Temperature_Rotor; /*!< Motor Temperature_Rotor */
  float  Angle;	
	
}DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
  
	DM_Motor_Control_Mode_Type_e	Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;   
	DM_Motor_Param_Range_Typedef Param_Range; 
	DM_Motor_Data_Typedef Data;   

}DM_Motor_Info_Typedef;


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Update the DJI motor Information
  */
extern void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor);

/**
  * @brief  Update the RMD motor Information
	*/
extern void RMD_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,RMD_L9025_Info_Typedef *RMD_Motor);
void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor);
extern DJI_Motor_Info_Typedef Motor_chassis[4];
extern DJI_Motor_Info_Typedef Pich_Motor;
extern DJI_Motor_Info_Typedef Magazine_Motor;
extern DJI_Motor_Info_Typedef Shoot_Motor[2];
extern DM_Motor_Info_Typedef DM_Yaw_Motor;
#endif //DEVICE_MOTOR_H        DJI_Motor_Info_Typedef Shoot_Motor[2]
