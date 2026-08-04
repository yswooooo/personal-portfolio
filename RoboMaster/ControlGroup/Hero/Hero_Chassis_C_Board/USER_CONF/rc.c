#include "rc.h"
#include "remote_ctrl.h"
#include "usart.h"
#include "dma.h"
#include "detect_task.h"

rc_info_t rc;//遥控与键盘

/*重置DMA*/
void DMA_ReStart(void)
{
	USART_Cmd(USART3,DISABLE);
	DMA_Cmd(DMA1_Stream1,DISABLE);
	DMA_SetCurrDataCounter(DMA1_Stream1,DBUS_BUFLEN);
	
	USART_ClearFlag(USART3,USART_FLAG_IDLE);
	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF2);
	
	USART_Cmd(USART3,ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);
	
}

/*USART3 中断函数*/
void USART3_IRQHandler(void)
{
	static uint16_t rbuf_size;
	if(USART_GetFlagStatus(USART3,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART3);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)//采用DMA双缓冲区进行接收数据，当使用memory0时
		{
			/*重置DMA*/
			DMA_Cmd(DMA1_Stream1,DISABLE);
			rbuf_size = DBUS_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
			if(rbuf_size == DBUS_BUFLEN*2)
			{
				remote_ctrl(&rc,dbus_buf[0]);
        err_detector_hook(REMOTE_CTRL_OFFLINE);
			}
			DMA_SetCurrDataCounter(DMA1_Stream1,DBUS_MAX_LEN);
			DMA1_Stream1->CR |= DMA_SxCR_CT;
			
			DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF2|DMA_FLAG_HTIF2);
			DMA_Cmd(DMA1_Stream1,ENABLE);	
		}
		else//当使用memory1时
		{
			/*重置DMA*/
			DMA_Cmd(DMA1_Stream1,DISABLE);
			rbuf_size = DBUS_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);
			if(rbuf_size == DBUS_BUFLEN*2)
			{
				remote_ctrl(&rc,dbus_buf[1]);
        err_detector_hook(REMOTE_CTRL_OFFLINE);
			}
			DMA_SetCurrDataCounter(DMA1_Stream1,DBUS_MAX_LEN);
			DMA1_Stream1 ->CR &= ~(DMA_SxCR_CT);
			
			DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF2|DMA_FLAG_HTIF2);
			DMA_Cmd(DMA1_Stream1,ENABLE);
		}
		USART_ClearFlag(USART3, USART_FLAG_IDLE);//清除空闲中断标志位
		USART_ClearITPendingBit(USART3,USART_FLAG_IDLE);
	}
}
