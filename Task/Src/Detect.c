#include "Detect.h"


#define CDC_RX_BUFFER_SIZE 64 
uint8_t cdc_rx_buffer[CDC_RX_BUFFER_SIZE]; // 循环缓存
uint16_t cdc_rx_index = 0;  

Upper_Info_Typedef HMI_Contrl_update; 

void MSG_TO_INFO(uint8_t *rx_buf, Upper_Info_Typedef *HMI_ctrl_update);

void Fill_Buffer(uint8_t *buffer, int startIndex, float data, uint8_t identifier, float scale, const uint8_t *numToChar)
{
int dataTemp = (int)fabs(data * scale);

    buffer[startIndex] = identifier;
    buffer[startIndex + 1] = numToChar[data >= 0 ? 0 : 1];

    for (int i = 0; i < 4; i++) {
        buffer[startIndex + 2 + i] = numToChar[(dataTemp / (int)pow(10, 3 - i)) % 10]; //pow(x,y) = x的y次方
    }

}

void MiniPC_Transmit_Info(float yaw_data,float pitch_data)
{
	const uint8_t check_bitl[3] = {0x61, 0x62, 0x63};
	const uint8_t Num_To_Char[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
	uint8_t send_data[13];
	float Yaw = yaw_data*57.295779513f;
	float Pitch = pitch_data*57.295779513f;
	Fill_Buffer(send_data, 0, Yaw, 0x61, 10.0f, Num_To_Char);
  Fill_Buffer(send_data, 6, Pitch, 0x62, 10.0f, Num_To_Char);
	send_data[12] = check_bitl[2];

}

void MiniPC_Recive_Info(uint8_t* Buf, uint32_t *Len)
{
for(uint32_t i = 0; i < *Len; i++){
    uint8_t byte = Buf[i];

    // 环形缓冲或简单线性缓存填充
    cdc_rx_buffer[cdc_rx_index++] = byte; 
    // 避免越界
    if (cdc_rx_index >= CDC_RX_BUFFER_SIZE) cdc_rx_index = 0; 
    // 检测是否接收到一帧完整协议（假设为固定17字节，且以'a'开头，'e'结尾）
    if (cdc_rx_index >= 17){
        // 查找帧头
      for (uint16_t j = 0; j <= cdc_rx_index - 17; j++){
        if (cdc_rx_buffer[j] == 'a' && cdc_rx_buffer[j + 16] == 'e'){
          // 找到完整帧，准备解析
          MSG_TO_INFO(&cdc_rx_buffer[j], &HMI_Contrl_update); 
          // 剩余数据前移（滑动窗口）
          memmove(cdc_rx_buffer, &cdc_rx_buffer[j + 17], cdc_rx_index - (j + 17));
          cdc_rx_index -= (j + 17);
          break;
        }
      }
    }
  }
	

}



void MSG_TO_INFO(uint8_t *rx_buf, Upper_Info_Typedef *HMI_ctrl_update)
{
//    if (rx_buf == NULL || HMI_ctrl_update == NULL)
//    {
//        return;
//    }

    static float last_yaw = 0.0f, last_pitch = 0.0f, last_target_tracking_state = 0.0f;

    float yaw = last_yaw, pitch = last_pitch, target_tracking_state = last_target_tracking_state;
    uint8_t yaw_flag = 0, pich_flag = 0;
    int re_flag = 0;

    // Validate re_flag
    if (rx_buf[0] == 'a')
    {
        if (rx_buf[1] == '0') re_flag = 1;
    }
	
	if(rx_buf[2] == 'b')
	{
		for(int i=3;i<=7;i++)
		{
			if(rx_buf[i]<'0'||rx_buf[i]>'9')
			{
				yaw_flag=1;//有问题标志位置为1
			}
		}	
	}		
	
	if(yaw_flag!=1 && re_flag==1) yaw		=(rx_buf[4]-'0')*100.0+(rx_buf[5]-'0')*10+ (rx_buf[6]-'0')*1.0+ (rx_buf[7]-'0')*0.1f;
		
	if (yaw_flag!=1 && (rx_buf[3] - '0')==1 && re_flag==1) yaw *= -1;
	

	if(rx_buf[8] == 'c')
	{
		for(int i=9;i<=13;i++)
		{
			if(rx_buf[i]<'0'||rx_buf[i]>'9')
			{
				pich_flag=1;//有问题标志位置为1
			}
		}	
	}		
	
	if(pich_flag!=1 && re_flag==1) pitch		=(rx_buf[10]-'0')*100.0+(rx_buf[11]-'0')*10+ (rx_buf[12]-'0')*1.0+ (rx_buf[13]-'0')*0.1f;
	if (pich_flag!=1 && (rx_buf[9] - '0')==1 && re_flag==1) pitch *= -1;
	
	if(rx_buf[14] == 'd' && rx_buf[16] == 'e' && re_flag==1) 
	{
		if((rx_buf[15]-'0')==1)target_tracking_state=1;
		else if((rx_buf[15]-'0')==2)target_tracking_state=2;
		else target_tracking_state=0;
	}
	if(re_flag==1)
	{
		HMI_ctrl_update->pitch_angle_get_temp = pitch/57.295779513f;
		HMI_ctrl_update->yaw_angle_get_temp = yaw/57.295779513f;
		HMI_ctrl_update->gimbal_state=target_tracking_state;
	}
	
	HMI_ctrl_update->receive_finish_flag = 1;
}


