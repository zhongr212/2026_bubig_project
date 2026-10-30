#ifndef CRC_H
#define CRC_H

#include "stdint.h"
#include "stdbool.h"

extern  bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern  bool Verify_CRC8_Check_Sum(unsigned char *pch_message, unsigned int dw_length);

extern uint16_t get_CRC16_check_sum(uint8_t *pch_message,uint32_t dw_length,uint16_t wCRC);

extern uint8_t Get_CRC8_Check_Sum(unsigned char *pch_message,unsigned int dw_length,unsigned char ucCRC8);

extern uint16_t Get_CRC16_Check_Sum(uint8_t *pch_message,uint32_t dw_length,uint16_t wCRC);

extern void append_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length);

extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);
#endif