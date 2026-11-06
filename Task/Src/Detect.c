#include "Detect.h"
#include "Magazine_task.h"
#include "Control_Gaimbal_task.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "CRC.h"
#include "usart.h"

__attribute__((section (".RAM_D1"))) uint8_t PC_TRAN_MultiRx_Buf[2][REC_DATA_NUM];


uint8_t send[SEND_DATA_NUM];       
uint8_t receive[REC_DATA_NUM];
receice_sta rx;


/*CRC初始化函数*/
//uint16_t crc16_Set(const uint8_t *data, size_t len)
//{
//    uint16_t crc = AVP_CRC_INIT;
//    size_t   i;
//    int      b;

//    if (data == NULL) {
//        return crc;
//    }
//    for (i = 0u; i < len; ++i) {
//        crc ^= (uint16_t)data[i];
//        for (b = 0; b < 8; ++b) {
//            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ AVP_CRC_POLY)
//                             : (uint16_t)(crc >> 1);
//        }
//    }
//    return crc;
//}

static uint32_t avp_unaligned_u32(const uint8_t *p)//uint8转uint32
{
    uint32_t v;
    memcpy(&v, p, sizeof(v));
    return v;
}

static float avp_unaligned_f32(const uint8_t *p)//uint8转float
{
    uint32_t bits = avp_unaligned_u32(p);
    float    v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}



#define AVP_RD_F32(buf, off) avp_unaligned_f32((const uint8_t *)(buf) + (off))
#define AVP_RD_U16(buf, off) ((uint16_t)(avp_unaligned_u32((const uint8_t *)(buf) + (off)) & 0xFFFFu))


/* ================= 通用校验 ================= */

//int crc_check(const uint8_t *frame, size_t len)
//{
//    if (frame == NULL || len < 3u || frame[0] != HEAD) {
//        return 0;
//    }
//    return (crc16_Set(frame, len - 2u) == AVP_RD_U16(frame, len - 2u)) ? 1 : 0;
//}


/* 填帧头并把 CRC 小端写到帧尾；返回整帧长度 */
static unsigned int Fomat_set(void *frame, size_t len)
{
    uint8_t *p = (uint8_t *)frame;
    uint16_t crc;

    p[0] = HEAD;
    crc  = Get_CRC16_Check_Sum(p, len - 2u,AVP_CRC_INIT);
    p[len - 2u] = (uint8_t)(crc & 0xFFu);
    p[len - 1u] = (uint8_t)((crc >> 8) & 0xFFu);
    return len;
}


/* ================= 组上行帧 ================= */

size_t Send_Init(uint8_t out[SEND_DATA_NUM],
                       float yaw, float pitch, float yaw_speed, float pitch_speed)
{
//    send_data f;
//		

//    f.header      = HEAD;
//    f.yaw         = yaw;
//    f.pitch       = pitch;
//    f.yaw_speed   = yaw_speed;
//    f.pitch_speed = pitch_speed;
//    f.checksum    = 0u;                结构体会补齐，造成超量呈
		
		out[0] = HEAD;

    memcpy(&out[1],  &yaw,        sizeof(float));
    memcpy(&out[5],  &pitch,      sizeof(float));
    memcpy(&out[9],  &yaw_speed,  sizeof(float));
    memcpy(&out[13], &pitch_speed, sizeof(float));

    out[17] = 0;
    out[18] = 0;
		
//    memcpy(out, &f, sizeof(f));      
    return Fomat_set(out, SEND_DATA_NUM);
}



/* ================= 解析下行帧 ================= */

int Receive_Task(uint8_t *buf, size_t len, receive_data *out)
{
    size_t off = 0u;

    if (buf == NULL || out == NULL || len != REC_DATA_NUM) {
        return 0;
    }
    if (!Verify_CRC16_Check_Sum(buf, len)) {          /* 先查 header 与 CRC 再取字段 */
        return 0;
    }
    out->header      = buf[off];            off += 1u;   /* 0  */
    out->yaw_angle   = AVP_RD_F32(buf, off); off += 4u;  /* 1  */
    out->yaw_speed   = AVP_RD_F32(buf, off); off += 4u;  /* 5  */
    out->yaw_acc     = AVP_RD_F32(buf, off); off += 4u;  /* 9  */
    out->pitch_angle = AVP_RD_F32(buf, off); off += 4u;  /* 13 */
    out->pitch_speed = AVP_RD_F32(buf, off); off += 4u;  /* 17 */
    out->pitch_acc   = AVP_RD_F32(buf, off); off += 4u;  /* 21 */
    out->fire_enable = buf[off];            off += 1u;   /* 25 */
    out->checksum    = AVP_RD_U16(buf, off);             /* 26 */
    return 1;
}



/* ================= 下行接收状态机 ================= */

void receive_init(receice_sta *rx, size_t frame_len)
{
    if (rx == NULL) {
        return;
    }
    memset(rx, 0, sizeof(*rx));
    rx->frame_len = frame_len;
}

int exam_rece_data(receice_sta *rx, const uint8_t *data, size_t n, uint8_t *out)
{
    size_t i, skip;

    if (rx == NULL || out == NULL) {
        return 0;
    }
    if (n > sizeof(rx->buf) - rx->len) {        /* 溢出：清空重新对齐 */
        rx->len = 0u;
        return 0;
    }
    if (data != NULL && n > 0u) {
        memcpy(rx->buf + rx->len, data, n);
        rx->len += n;
    }//传值操作，保证在有值的情况下传值，跳跃没有接到帧头而储存的部分

    for (;;) {
        if (rx->len == 0u) {
            return 0;
        }
        /* 丢弃开头直到 0xA5 */
        if (rx->buf[0] != HEAD) {
            for (i = 1u; i < rx->len && rx->buf[i] != HEAD; ++i) { }
            skip = i;
            if (skip < rx->len) {
                memmove(rx->buf, rx->buf + skip, rx->len - skip);
            }
            rx->len -= skip;
            if (rx->len == 0u) {
                return 0;
            }
        }
        if (rx->len < rx->frame_len) {          /* 不足一整帧，等后续字节 */
            return 0;
        }
        if (Verify_CRC16_Check_Sum(rx->buf, rx->frame_len)) { /* CRC 正确：提交并删除整帧 */
            memcpy(out, rx->buf, rx->frame_len);
            rx->frames_ok++;
            memmove(rx->buf, rx->buf + rx->frame_len, rx->len - rx->frame_len);
            rx->len -= rx->frame_len;
            return 1;                           /* 一次最多交出一帧 */
        }
        /* CRC 错误：只丢弃当前帧头，继续搜索下一个 0xA5 */
        rx->frames_bad_crc++;
        memmove(rx->buf, rx->buf + 1u, rx->len - 1u);
        rx->len -= 1u;
    }
}

/* ================= 平台接口（按实际平台实现） ================= */

/* 上行帧发送周期：10ms 即 100Hz，与视觉下行 100Hz 对齐 */
#define SEND_PERIOD_MS   10u

void UART7_Send_Blocking(uint8_t *pData, uint16_t Size)
{
    while(HAL_UART_Transmit_DMA(&huart7,pData, Size) != HAL_OK)
    {
			
        osDelay(1);                             /* 已转 FreeRTOS，用 osDelay */
    }
}


/* 接收：一次把已到达的字节读出来（DMA/中断环形缓冲取数据，或阻塞读一批）。
 * 返回本次读到的字节数，0 表示暂无数据。不要在这里拼帧，交给状态机。 */
//static size_t uart_read(uint8_t *buf, size_t cap)
//{
//    uint16_t len = (uint16_t)cap;          /* 取帧函数会把实际帧长写回这里 */

//    if (UART7_DMA_GetRxFrame(buf, &len))
//    {
//        return (size_t)len;
//    }
//    return 0u;                             /* 无新帧必须返回 0，不能返回未初始化的栈值 */
//}

/* 把视觉下发的目标值交给云台/底盘控制：角度与角速度、加速度均为参考值，
 * chassis_vx/vy 仅在哨兵帧存在（步兵传 0）。 */
static void apply_target(float yaw_angle, float yaw_speed, float yaw_acc,
                         float pitch_angle, float pitch_speed, float pitch_acc,
                         uint8_t fire_enable)
{
    Control_Gimbal_Info.upper_data.yaw_angle = yaw_angle; 
		Control_Gimbal_Info.upper_data.yaw_speed = yaw_speed; 
		Control_Gimbal_Info.upper_data.yaw_acc = yaw_acc;
    Control_Gimbal_Info.upper_data.pitch_angle = pitch_angle;
		Control_Gimbal_Info.upper_data.pitch_speed = pitch_speed;
		Control_Gimbal_Info.upper_data.pitch_acc = pitch_acc;
		Control_Magazine_Info.fire_enable = fire_enable;
}

/* 从 IMU / 云台反馈取角度与角速度，必须换算成弧度、rad/s. */
static void read_gimbal_state(float *yaw, float *pitch,
                              float *yaw_speed, float *pitch_speed)
{
    *yaw = Control_Gimbal_Info.Measure.Yaw_IMU_Angle; *pitch = Control_Gimbal_Info.Measure.Pitch_Motor_Angle;
    *yaw_speed =Control_Gimbal_Info.Measure.Yaw_IMU_Gyro; *pitch_speed = Control_Gimbal_Info.Measure.Pitch_Motor_Gyro;

}

/* 发送：STM32 -> HAL_UART_Transmit(&huart, buf, len, 超时)；
 *       RT-Thread -> rt_device_write(dev, 0, buf, len)；Linux -> write(fd, ...)。 */
static void uart_write(uint8_t *buf, size_t len)
{
	UART7_Send_Blocking(buf,len);
 
}

void rec_data(uint8_t* chunk,uint16_t leng)
{
 if (exam_rece_data(&rx, chunk, leng, receive) != 0) {

      receive_data d;
     if (Receive_Task(receive, sizeof(receive), &d) != 0) {
         apply_target(d.yaw_angle, d.yaw_speed, d.yaw_acc,
                             d.pitch_angle, d.pitch_speed, d.pitch_acc,
                              d.fire_enable);
       }

      }
}

void Detect_Task(void)
{

		receive_init(&rx, sizeof(receive));         /* 步兵 28 / 哨兵 36 */
    memset(send, 0, sizeof(send));

    uint32_t send_tick = osKernelSysTick();         /* 上一次上行发送时刻 */
    uint32_t loop_tick = osKernelSysTick();         /* 本轮循环起点 */

    for (;;) {
        uint8_t chunk[32];
        size_t  leng;

        /* --- 1) 收视觉下行帧：读一批字节喂给状态机，凑齐一帧再解帧 --- */
//        leng = uart_read(chunk, sizeof(chunk));


        /* --- 2) 回发上行帧：每 10ms 一帧（100Hz）。
         *     读数与收帧每轮都做，只有到点才发，发送周期不受接收耗时影响 --- */
        
            uint32_t now = osKernelSysTick();

            if ((now - send_tick) >= SEND_PERIOD_MS)
            {
                float yaw, pitch, yaw_speed, pitch_speed;
                size_t len;

                send_tick = now;

                read_gimbal_state(&yaw, &pitch, &yaw_speed, &pitch_speed);

                len = Send_Init(send, yaw, pitch, yaw_speed, pitch_speed);

                uart_write(send, len);
            }
        

        /* --- 3) 让出 CPU：扣掉本轮已用时间，使循环周期稳定在 10ms --- */
        
            uint32_t elapsed = osKernelSysTick() - loop_tick;

            osDelay((elapsed < SEND_PERIOD_MS) ? (SEND_PERIOD_MS - elapsed) : 1u);
            loop_tick = osKernelSysTick();
        
    }
}
