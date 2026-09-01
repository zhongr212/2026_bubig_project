#include "Motor.h"



static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *,float ,uint16_t );

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

static int float_to_uint(float x, float x_min, float x_max, int bits);
/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  */
float encoder_to_angle(Motor_GeneralInfo_Typedef *,float ,uint16_t );
/** 
  * @brief  Judge the DJI Motor state
  */
static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *);
//速度
DJI_Motor_Info_Typedef Motor_chassis[4] =
{
 [0] = {.CANFrame.RxStdId=0x201,
	      .CANFrame.TxStdId=0x200,
	      .Type=DJI_AGV3508,
},
 [1] = {.CANFrame.RxStdId=0x202,
	      .CANFrame.TxStdId=0x200,
	      .Type=DJI_AGV3508,
},
 [2] = {.CANFrame.RxStdId=0x203,
	      .CANFrame.TxStdId=0x200,
	      .Type=DJI_AGV3508,
},
 [3] = {.CANFrame.RxStdId=0x204,
	      .CANFrame.TxStdId=0x200,
	      .Type=DJI_AGV3508,
},
 
};
//角度
DM_Motor_Info_Typedef DM_Yaw_Motor ={

		  .Control_Mode = MIT,
			.Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 30.f,
			   .T_MAX = 10.f		
			},
		  .FDCANFrame = {
				 .TxStdId = 0x4,
				 .RxStdId = 0x14,//接受电机
			},
		
};//后期依据实际更改

DJI_Motor_Info_Typedef Pich_Motor={
.CANFrame.RxStdId=0x205,
.CANFrame.TxStdId=0x1FE,
.Type=DJI_GM6020,
};

DJI_Motor_Info_Typedef Magazine_Motor={
.CANFrame.RxStdId=0x207,
.CANFrame.TxStdId=0x1FF,
.Type=DJI_M2006,
};

DJI_Motor_Info_Typedef Shoot_Motor[2] =
{
 [0] = {.CANFrame.RxStdId=0x201,
	      .CANFrame.TxStdId=0x200,
	      .Type=DJI_AGV3508,
},
 [1] = {.CANFrame.RxStdId=0x202,
	      .CANFrame.TxStdId=0x200,
	      .Type=DJI_AGV3508,
},
 
};



////////////////////////DJI motor ////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* check the StdId */
	if(*StdId != DJI_Motor->CANFrame.RxStdId) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.Temperature = rxBuf[6];
	DJI_Motor->Data.Encoder  = ((int16_t)rxBuf[0] << 8 | (int16_t)rxBuf[1]);
	DJI_Motor->Data.Velocity = ((int16_t)rxBuf[2] << 8 | (int16_t)rxBuf[3]);
	DJI_Motor->Data.Current  = ((int16_t)rxBuf[4] << 8 | (int16_t)rxBuf[5]);
	
	/* Judge the motor error	*/
	DJI_Motor_ErrorHandler(DJI_Motor);

  /* update the txframe id and index */
  if(DJI_Motor->Data.Initlized != true)
  {
    if(DJI_Motor->CANFrame.RxStdId > DJI_RxFrame_MIDDLE)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_HIGH;
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_RxFrame_MIDDLE - 0x01U);
    }
    else if(DJI_Motor->CANFrame.RxStdId > DJI_TxFrame_LOW)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_LOW; 
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_TxFrame_LOW - 0x01U);
    }
  }
	//？
	/* transform the encoder to anglesum */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.Angle = encoder_to_angle(&DJI_Motor->Data,1.f,8192);//360
		break;
		case DJI_M3508:
			DJI_Motor->Data.Angle = encoder_to_angle(&DJI_Motor->Data,3591.f/187.f,8192);
		break;
		case DJI_M2006:
			DJI_Motor->Data.Angle = encoder_to_anglesum(&DJI_Motor->Data,36.f,8192);
		break;
			case DJI_AGV3508:
      DJI_Motor->Data.Angle = encoder_to_angle(&DJI_Motor->Data, 8.f, 8192);
      break;
		default:break;
	}
}
//------------------------------------------------------------------------------


/**
  * @brief  transform the encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info        pointer to a Motor_GeneralInfo_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXencoder   the specified motor max encoder number
  * @retval anglesum
  */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{
  float res1 = 0,res2 =0;
  
  if(Info == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Info->Initlized != true)
  {
    /* update the last encoder */
    Info->Last_Encoder = Info->Encoder;

    /* reset the angle */
    Info->Angle = 0;

    /* Set the init flag */
    Info->Initlized = true;
  }
  
  /* get the possiable min encoder err */
  if(Info->Encoder < Info->Last_Encoder)
  {
      res1 = Info->Encoder - Info->Last_Encoder + MAXencoder;
  }
  else if(Info->Encoder > Info->Last_Encoder)
  {
      res1 = Info->Encoder - Info->Last_Encoder - MAXencoder;
  }
  res2 = Info->Encoder - Info->Last_Encoder;
  
  /* update the last encoder */
  Info->Last_Encoder = Info->Encoder;
  
  /* transforms the encoder data to tolangle */
	if(fabsf(res1) > fabsf(res2))
	{
		Info->Angle += (float)res2/(MAXencoder*torque_ratio)*360.f;
	}
	else
	{
		Info->Angle += (float)res1/(MAXencoder*torque_ratio)*360.f;
	}
  
  return Info->Angle;
}
//------------------------------------------------------------------------------

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }
  
  float len = maxValue - minValue;    

  if (Input > maxValue)
  {
      do{
          Input -= len;
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;
      }while (Input < minValue);
  }
  return Input;
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  * @param  *Info        pointer to a Motor_GeneralInfo_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXencoder   the specified motor max encoder number
  * @retval angle
  */
float encoder_to_angle(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{	
   float encoder_err = 0.f;
  
  /* check the motor init */
  if(Info->Initlized != true)
  {
    /* update the last encoder */
    Info->Last_Encoder = Info->Encoder;

    /* reset the angle */
    Info->Angle = Info->Encoder/(MAXencoder*torque_ratio)*360.f;

    /* config the init flag */
    Info->Initlized = true;
  }
  
  encoder_err = Info->Encoder - Info->Last_Encoder;
  
  /* 0 -> MAXencoder */		
  if(encoder_err > MAXencoder*0.5f)
  {
    Info->Angle += (float)(encoder_err - MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  /* MAXencoder-> 0 */		
  else if(encoder_err < -MAXencoder*0.5f)
  {
    Info->Angle += (float)(encoder_err + MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  else
  {
    Info->Angle += (float)(encoder_err)/(MAXencoder*torque_ratio)*360.f;
  }
  
  /* update the last encoder */
  Info->Last_Encoder = Info->Encoder;
  
  /* loop constrain */
  Info->Angle = f_loop_constrain(Info->Angle,-180.f,180.f);

  return Info->Angle;

}
//------------------------------------------------------------------------------

/** 
  * @brief  Judge the DJI Motor state
  * @param  *DJI_Motor pointer to a DJI_Motor_Info_Typedef structure that contains
  *                    the configuration information for the specified motor.  
  * @retval None
  */
static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* Judge the DJI motor temperature */
	if(DJI_Motor->Data.Temperature > 80)
	{
    DJI_Motor->ERRORHandler.ErrorCount++;

    if(DJI_Motor->ERRORHandler.ErrorCount > 200)
    {
      DJI_Motor->ERRORHandler.Status = MOTOR_OVER_TEMPERATURE;
      DJI_Motor->ERRORHandler.ErrorCount = 0;
    }
	}
  else
	{
    DJI_Motor->ERRORHandler.ErrorCount = 0;	
	}
}

//------------------------------------------------------------------------------

/**
  * @brief  Update the DM_Motor Information
  * @param  Identifier:  pointer to the specifies the standard identifier.
  * @param  Rx_Buf:  pointer to the can receive data
  * @param  DM_Motor: pointer to a DM_Motor_Info_Typedef structure that contains the information of DM_Motor
  * @retval None
  */
void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor)
{	 
	if(*Identifier != DM_Motor->FDCANFrame.RxStdId) return;
	
	  DM_Motor->Data.State = Rx_Buf[0]>>4;
		DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) <<8) | ((uint16_t)(Rx_Buf[2]));
		DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) <<4) | ((uint16_t)(Rx_Buf[4])>>4);
		DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4]&0xF) <<8) | ((uint16_t)(Rx_Buf[5]));
		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_Motor->Param_Range.T_MAX,DM_Motor->Param_Range.T_MAX,12);
		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_Motor->Param_Range.P_MAX,DM_Motor->Param_Range.P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_Motor->Param_Range.V_MAX,DM_Motor->Param_Range.V_MAX,12);
	  DM_Motor->Data.Angle = DM_Motor->Data.Position*RadiansToDegrees;

    DM_Motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
		DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);

}



static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
	
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
	
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}