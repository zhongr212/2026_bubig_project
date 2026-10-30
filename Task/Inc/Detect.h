#ifndef __DETECT_H
#define __DETECT_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAD          0xA5u   //帧头
#define AVP_CRC_INIT        0xFFFFu
#define AVP_CRC_POLY        0xA001u

#define SEND_DATA_NUM   19u   /* 步兵程序上行 */
#define REC_DATA_NUM 28u   /* 步兵程序下行 */
#define REC_DATA_NUM_USART 56u   


#define AVP_DOWN_RX_CAP       128u  /* 下行接收缓存 */

typedef struct {                    /* 步兵上行 19B */
    uint8_t  header;                /* 0xA5        */
    float    yaw;                   /* 小 yaw, rad */
    float    pitch;                 /* rad         */
    float    yaw_speed;             /* rad/s       */
    float    pitch_speed;           /* rad/s       */
    uint16_t checksum;              /* CRC16，小端 */
} send_data;



typedef struct {                    /* 步兵下行 28B（无底盘速度字段） */
    uint8_t  header;                /* 偏移 0  */
    float    yaw_angle;             /* 偏移 1  目标小 yaw, rad */
    float    yaw_speed;             /* 偏移 5  rad/s   */
    float    yaw_acc;               /* 偏移 9  rad/s^2 */
    float    pitch_angle;           /* 偏移 13 rad     */
    float    pitch_speed;           /* 偏移 17 rad/s   */
    float    pitch_acc;             /* 偏移 21 rad/s^2 */
    uint8_t  fire_enable;           /* 偏移 25 0/1/2   */
    uint16_t checksum;              /* 偏移 26 */
} receive_data;


/* ---------------- 下行接收状态机（文档第 8 节） ---------------- */

typedef struct {
    uint8_t  buf[AVP_DOWN_RX_CAP];
    size_t   len;
    size_t   frame_len;         /* 步兵 28 / 哨兵 36 */
    uint32_t frames_ok;         /* 累计成功帧数     */
    uint32_t frames_bad_crc;    /* 累计 CRC 错误次数 */
} receice_sta;

int crc_check(const uint8_t *frame, size_t len);


extern uint8_t PC_TRAN_MultiRx_Buf[2][REC_DATA_NUM];
void rec_data(uint8_t* chunk,uint16_t leng);
#endif