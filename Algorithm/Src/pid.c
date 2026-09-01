/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include "arm_math.h"
/* Includes ------------------------------------------------------------------*/

/**
 * @brief 初始化PID参数.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @Param Param: 指向PID参数的浮点数指针，包含PID参数信息.
 * @retval PID错误状态.
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID,float Param[PID_PARAMETER_NUM])
{
    //判断PID类型和参数是否为空 若为空 返回PID_FAILED_INIT（初始化失败）
   //?ж?PID?????????????? ????? ????PID_FAILED_INIT???????????
    if(PID->Type == PID_Type_None || Param == NULL)
    {
      return PID_FAILED_INIT;
    }
    
    //?????PID????
    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];

		PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];

    // ???????????? (????PID_ANTI_WINDUP????)
    PID->Param.I_Variable_Speed_A = Param[7];
    PID->Param.I_Variable_Speed_B = Param[8];
    
    // ???????
    PID->Param.KF = Param[9];
    // ???????????
    memset(PID->Target_Buffer, 0, sizeof(PID->Target_Buffer));
    PID->Target_Index = 0;

    if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f) 
		LowPassFilter1p_Init(&PID->Dout_LPF,PID->Param.Alpha);
		

    //???PID???????
    PID->ERRORHandler.ErrorCount = 0;

		//????PID_ERROR_NONE???????????
    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief 清除PID计算值，将所有输出赋0.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @retval 无.
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
	//将所有输出赋0
	memset(PID->Err,0,sizeof(PID->Err));
	PID->Integral = 0;
		
	PID->Pout = 0;
	PID->Iout = 0;
	PID->Dout = 0;
	PID->Output = 0;
}
//------------------------------------------------------------------------------

static float GetSmoothedTargetDelta(PID_Info_TypeDef *PID) {
    float delta_sum = 0.0f;
    for (uint8_t i = 0; i < FF_WINDOW_SIZE - 1; i++) {
        delta_sum += (PID->Target_Buffer[(PID->Target_Index + i + 1) % FF_WINDOW_SIZE] - 
                    PID->Target_Buffer[(PID->Target_Index + i) % FF_WINDOW_SIZE]);
    }
    return delta_sum / (FF_WINDOW_SIZE - 1);
}

/**
 * @brief 初始化PID控制器.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @Param Type: PID控制器类型.
 * @Param Param:指向PID参数的浮点数指针，包含PID参数信息.
 * @retval PID错误状态.
 */
void PID_Init(PID_Info_TypeDef *PID,PID_Type_e Type,float Param[PID_PARAMETER_NUM])
{
	
		PID->Type = Type;

		PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

		PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}
//------------------------------------------------------------------------------


/**
  * @brief 判断PID错误状态
  * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
  * @retval 无.
  */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
		/* Judge NAN/INF */
		if(isnan(PID->Output) == true || isinf(PID->Output)==true)
		{
				PID->ERRORHandler.Status = PID_CALC_NANINF;
		}
}
//------------------------------------------------------------------------------

/**
  * @brief  PID控制器计算函数
  * @Param  *PID pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @Param  Target  Target for the PID controller
  * @Param  Measure Measure for the PID controller
  * @retval the PID Output
  */
float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure)
{		
  /* update the PID error status */
  PID_ErrorHandle(PID);
  if(PID->ERRORHandler.Status != PID_ERROR_NONE)
  {
    PID->PID_Calc_Clear(PID);
    return 0;
  }
  
  /* update the target/measure */
  PID->Target =  Target;
  PID->Measure = Measure;

  /* update the error */
	PID->Err[2] = PID->Err[1];
	PID->Err[1] = PID->Err[0];
	PID->Err[0] = PID->Target - PID->Measure;
		
	 float abs_error = fabsf(PID->Err[0]);

    if (abs_error < PID->Param.Deadband) {
        PID->Err[0] = 0.0f;
        abs_error = 0.0f;
    } else if (PID->Err[0] > 0.0f) {
        PID->Err[0] -= PID->Param.Deadband;
    } else {
        PID->Err[0] += PID->Param.Deadband;
    }
    

    abs_error = fabsf(PID->Err[0]);
  if(fabsf(PID->Err[0]) >= 0)
  {
		    switch (PID->Type) {
        case PID_POSITION: {
     //位置式
            PID->Pout = PID->Param.KP * PID->Err[0];
            if (PID->Param.KI != 0) {
                PID->Integral += PID->Err[0];
                VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);
                PID->Iout = PID->Param.KI * PID->Integral;
            } else {
                PID->Iout = 0;
            }
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);
            if (PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
            break;
        }
        case PID_VELOCITY: {
            //增量式
            PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);
            PID->Iout = PID->Param.KI * PID->Err[0];
            PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f * PID->Err[1] + PID->Err[2]);
            if (PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }
            PID->Output += PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
            break;
        }
        case PID_ANGLE: {
            //新增角度转换
            float angle_error = PID->Err[0];
            // ???????
            while (angle_error > 180.0f) angle_error -= 360.0f;
            while (angle_error < -180.0f) angle_error += 360.0f;
            PID->Err[0] = angle_error;

            PID->Pout = PID->Param.KP * PID->Err[0];
            if (PID->Param.KI != 0) {
                PID->Integral += PID->Err[0];
                VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);
                PID->Iout = PID->Param.KI * PID->Integral;
            } else {
                PID->Iout = 0;
            }
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);
            if (PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
            break;
        }
        case PID_ANTI_WINDUP: {
           //变速积分B - A
            float set_ratio = 0;
            PID->Pout = PID->Param.KP * PID->Err[0];
            
            if (abs_error <= PID->Param.I_Variable_Speed_B)
            {
            set_ratio = 1 ;
            }
            else if(abs_error > PID->Param.I_Variable_Speed_B && abs_error <= (PID->Param.I_Variable_Speed_B+PID->Param.I_Variable_Speed_A))
            {
            set_ratio = (PID->Param.I_Variable_Speed_A - abs_error + PID->Param.I_Variable_Speed_B)/PID->Param.I_Variable_Speed_A;
            }
            else if(abs_error > (PID->Param.I_Variable_Speed_B+PID->Param.I_Variable_Speed_A))
            {
            set_ratio = 0;
            }
            PID->Iout += PID->Param.KI * PID->Err[0]*set_ratio;
            PID->Dbuf[2] = PID->Dbuf[1];
            PID->Dbuf[1] = PID->Dbuf[0]; 
            PID->Dbuf[0] = (PID->Err[0]-PID->Err[1]);  
            PID->Dout = PID->Param.KD * (PID->Dbuf[0] * 0.85F + PID->Dbuf[1]*0.1f + PID->Dbuf[1]*0.05f);
            VAL_LIMIT(PID->Iout,-PID->Param.LimitIntegral, PID->Param.LimitIntegral);
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            break;
        }
				case PID_ANTI_WINDUP_DEGREE: {
           //变速积分+角度转化
            float set_ratio = 0;
						 /* ??????PID */
            float angle_error = PID->Err[0];
            // ???????
            if (angle_error > 0.99f*PI) angle_error -= (2*PI);
            else if  (angle_error < -0.99f*PI) angle_error += (2*PI);
            PID->Err[0] = angle_error;
						PID->Pout = PID->Param.KP * PID->Err[0];
            
            if (abs_error <= PID->Param.I_Variable_Speed_B)
            {
            set_ratio = 1 ;
            }
            else if(abs_error > PID->Param.I_Variable_Speed_B && abs_error <= (PID->Param.I_Variable_Speed_B+PID->Param.I_Variable_Speed_A))
            {
            set_ratio = (PID->Param.I_Variable_Speed_A - abs_error + PID->Param.I_Variable_Speed_B)/PID->Param.I_Variable_Speed_A;
            }
            else if(abs_error > (PID->Param.I_Variable_Speed_B+PID->Param.I_Variable_Speed_A))
            {
            set_ratio = 0;
            }
            PID->Iout += PID->Param.KI * PID->Err[0]*set_ratio;
            PID->Dbuf[2] = PID->Dbuf[1];
            PID->Dbuf[1] = PID->Dbuf[0]; 
						
						//
						float delta_error = PID->Err[0] - PID->Err[1];
						if (delta_error > PI) delta_error -= 2*PI;
						else if (delta_error < -PI) delta_error += 2*PI;

						PID->Dbuf[0] = delta_error;
						
            PID->Dout = PID->Param.KD * (PID->Dbuf[0] * 0.85F + PID->Dbuf[1]*0.1f + PID->Dbuf[1]*0.05f);
            VAL_LIMIT(PID->Iout,-PID->Param.LimitIntegral, PID->Param.LimitIntegral);
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            break;
        }
        case PID_FEEDFORWARD: {
            //增加前馈
            PID->Pout = PID->Param.KP * PID->Err[0];
            
            if (PID->Param.KI != 0) {
                PID->Integral += PID->Err[0];
                VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);
                PID->Iout = PID->Param.KI * PID->Integral;
            } else {
                PID->Iout = 0;
            }
            
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);
            if (PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }
            

            PID->Fout = PID->Param.KF * GetSmoothedTargetDelta(PID);
            PID->Output = PID->Pout + PID->Iout + PID->Dout + PID->Fout;
            break;
          }        
        default:
           
            PID->Output = 0;
            break;
    }
  }
	else
	{
		PID->Output = 0;
	}

  return PID->Output;
}
//------------------------------------------------------------------------------

