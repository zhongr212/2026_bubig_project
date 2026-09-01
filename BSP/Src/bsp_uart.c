#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"
#include <stdio.h>
#include <stdarg.h>
#include "motor.h"
#include "Referee_System.h"
#include "Control_Task.h"
#include "CAN_Task.h"
 void usart_printf(const char *fmt,...){
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
	  __attribute__((section (".AXI_SRAM"))) static uint8_t tx_buf[256] = {0};
    len = vsnprintf((char *)tx_buf,sizeof(tx_buf) ,(char*)fmt, ap);
    
    va_end(ap);

    //HAL_UART_Transmit_DMA(&huart7,(uint8_t*)tx_buf, len);
}

__attribute__((section (".RAM_D1"))) uint8_t Rx_Data[2][40] = {0};
static float bit8TOfloat32_commit(uint8_t change_info[4]);

static void DMA_XferCpltCallback( DMA_HandleTypeDef * hdma);

static void DMA_XferHalfCpltCallback( DMA_HandleTypeDef * hdma);

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
		
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);//取出相应部分

  /* Configure the source memory Buffer address  */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

  /* Configure the destination memory Buffer address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);	
	
	
}

void BSP_USART_Init(void){

		
	USART_RxDMA_MultiBuffer_Init(&huart1,(uint32_t *)Referee_System_Info_MultiRx_Buf[0],(uint32_t *)Referee_System_Info_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);

  USART_RxDMA_MultiBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],RC_FRAME_LENGTH);
    
	HAL_Delay(500);


}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
			{
					//Disable DMA 
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
					/* reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

					if(Size == RC_FRAME_LENGTH)
					{
						SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
					}
					
			}
			/* Current memory buffer used is Memory 1 */
			else
			{
					//Disable DMA 
					__HAL_DMA_DISABLE(huart->hdmarx);
				
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
					/* reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

					if(Size == RC_FRAME_LENGTH)
					{
						SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
					}
					
					
			}
			
}

static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)//判断缓存区0或1
	{
		
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;//交替使用缓存区
				
  				__HAL_DMA_SET_COUNTER(huart->hdmarx,16);

				  if(Size >= 10){
						
						  Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[0]);
				
				      memset(Referee_System_Info_MultiRx_Buf[0],0,REFEREE_RXFRAME_LENGTH);//清空

				      __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH);//设置下一轮 DMA 最多接收 REFEREE_RXFRAME_LENGTH + 1 字节。
          }
					
					
	}
	else
	{
					__HAL_DMA_DISABLE(huart->hdmarx);
				
					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				

		    if(Size >= 10){
        
			   Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[1]);
				
				 memset(Referee_System_Info_MultiRx_Buf[1],0,REFEREE_RXFRAME_LENGTH);

				 __HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_RXFRAME_LENGTH);
      }
					
					
	}
  
}

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
			{
					//Disable DMA 
					__HAL_DMA_DISABLE(huart->hdmarx);

					((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
					/* reset the receive count */
				
  				__HAL_DMA_SET_COUNTER(huart->hdmarx,80);				
			}
			/* Current memory buffer used is Memory 1 */
			else
			{
					//Disable DMA 
					__HAL_DMA_DISABLE(huart->hdmarx);
				
					 ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				
					/* reset the receive count */
					__HAL_DMA_SET_COUNTER(huart->hdmarx,80);

			}


}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{	
			 if(huart == &huart1){
	
		  USER_USART1_RxHandler(huart,Size);
			
	}
	
	
	 if(huart == &huart5){
	
		USER_USART5_RxHandler(huart,Size);
	} 
	 
	 if(huart == &huart3){
	 
      USER_USART3_RxHandler(huart,Size);
			
	}
	
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

void USART_Vofa_Justfloat_Transmit(float SendValue1, float SendValue2, float SendValue3)
{

	uint8_t Tx_Buf[16];
	uint8_t *SendValue1_Pointer, *SendValue2_Pointer, *SendValue3_Pointer;

	SendValue1_Pointer = (uint8_t *)&SendValue1;
	SendValue2_Pointer = (uint8_t *)&SendValue2;
	SendValue3_Pointer = (uint8_t *)&SendValue3;

	Tx_Buf[0] = *SendValue1_Pointer;
	Tx_Buf[1] = *(SendValue1_Pointer + 1);
	Tx_Buf[2] = *(SendValue1_Pointer + 2);
	Tx_Buf[3] = *(SendValue1_Pointer + 3);
	Tx_Buf[4] = *SendValue2_Pointer;
	Tx_Buf[5] = *(SendValue2_Pointer + 1);
	Tx_Buf[6] = *(SendValue2_Pointer + 2);
	Tx_Buf[7] = *(SendValue2_Pointer + 3);
	Tx_Buf[8] = *SendValue3_Pointer;
	Tx_Buf[9] = *(SendValue3_Pointer + 1);
	Tx_Buf[10] = *(SendValue3_Pointer + 2);
	Tx_Buf[11] = *(SendValue3_Pointer + 3);
	Tx_Buf[12] = 0x00;
	Tx_Buf[13] = 0x00;
	Tx_Buf[14] = 0x80;
	Tx_Buf[15] = 0x7F;
	HAL_UART_Transmit(&huart10, Tx_Buf, sizeof(Tx_Buf), 10);
}









