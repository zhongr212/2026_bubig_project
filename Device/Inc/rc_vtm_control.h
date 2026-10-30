#ifndef RC_VTM_CONTROL_H
#define RC_VTM_CONTROL_H

#include "stdint.h"
#include "stdbool.h"
/* ----------------------- 摇杆归一化参数 ------------------------ */
#define RC_CH_VALUE_MIN        ((uint16_t)364)
//#define RC_CH_VALUE_OFFSET     ((uint16_t)1024)
#define RC_CH_VALUE_MAX        ((uint16_t)1684)

#define RC_CH_SBUS_BUF_NUM		42u

#define RC_CH_VALUE_LENGTH		21u

typedef struct 
{
	struct
	{
			int16_t ch[4];
			uint8_t s;
			uint8_t stop;
			uint8_t use_botton_L;
			uint8_t use_botton_R;
			int16_t wheel;
			uint8_t shoot;
	}rc;
	
	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t left;
		uint8_t middle;
		uint8_t right;
	
	}mouse;	
	
	uint16_t key;
	
	uint16_t crc16;
	
}vtm_rc;


extern  uint8_t VTM_SBUS_MultiRx_Buf[2][RC_CH_VALUE_LENGTH] ;
extern vtm_rc vtm_rc_data;
extern void vt13_rc(uint8_t* VT13_buf,vtm_rc* rc_ctrl);
















#endif