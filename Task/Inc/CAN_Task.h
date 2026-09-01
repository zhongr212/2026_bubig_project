#ifndef CAN_TASK_H
#define CAN_TASK_H
#include "cmsis_os.h"
#include "Referee_System.h"
#define MESSAGE_PACKED __attribute__((packed))
typedef struct 
{ 
 uint32_t sentry_cmd;  
} sentry_cmd_t; 


typedef struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
    uint16_t cmd_id;
} frame_header_t;


#pragma pack(1)
typedef struct {

    struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq;
    uint8_t crc8;

} header;

		uint16_t cmd_id;

    struct{ 
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint32_t sentry_cmd;
} data;

  uint16_t crc16;

} Sentry_Tx_Referee_t ;

#pragma pack()

/**
 * @brief  typedef enum that CMD of DM_Motor .
 */
typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;


typedef struct 
{ 
 struct{
 uint8_t time;
 uint8_t symbol;
 uint16_t status;
 uint16_t current_hp;
 uint16_t last_current_hp;
 } hurt;  
 
 uint8_t out_fight;
}sentry_status_t; 

extern sentry_status_t sentry_status; 
extern sentry_cmd_t sentry_cmd;
extern void Referee_System_Sentry();
extern Sentry_Tx_Referee_t Sentry_Tx_Referee;
#endif
