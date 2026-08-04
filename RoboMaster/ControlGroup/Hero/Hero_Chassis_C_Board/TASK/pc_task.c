#include "pc_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"

extern TaskHandle_t pc_rx_Task_Handle;
UBaseType_t pc_tx_stack_surplus;
UBaseType_t pc_rx_stack_surplus;

void pc_tx_task(void *parm)
{
	uint32_t pc_wake_time = osKernelSysTick();
  while(1)
  {
	pc_send_data_packet_pack();
    send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
    pc_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    vTaskDelayUntil(&pc_wake_time, 20);
  }
}

#ifdef USE_USB_OTG_FS
extern fifo_s_t  pc_rxdata_fifo;
extern uint8_t USB_USART_RX_BUF[64];
extern uint32_t USB_USART_RX_STA;
#endif
void pc_rx_task(void *parm)
{
	#ifndef USE_USB_OTG_FS
	uint32_t Signal;
	BaseType_t STAUS;
	#endif // USE_USB_OTG_FS
	
  while(1)
  {
		#ifdef USE_USB_OTG_FS


			fifo_s_puts(&pc_rxdata_fifo, USB_USART_RX_BUF, USB_USART_RX_STA);
			unpack_fifo_data(&pc_unpack_obj, UP_REG_ID); 

		#else
			STAUS = xTaskNotifyWait((uint32_t) NULL, 
															(uint32_t) PC_UART_IDLE_SIGNAL, 
															(uint32_t *)&Signal, 
															(TickType_t) portMAX_DELAY );
			if(STAUS == pdTRUE)
			{
				if(Signal & PC_UART_IDLE_SIGNAL)
				{
					
					dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT);
					unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
   
				}
			}
		#endif
    pc_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		
  }
}


/*USART1 中断函数*/
void USART1_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
	if(USART_GetFlagStatus(USART1,USART_FLAG_IDLE) != RESET 
		 && USART_GetITStatus(USART1,USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART1);		
		USART_ClearFlag(USART1, USART_FLAG_IDLE);//清除空闲中断标志位
    
    err_detector_hook(PC_SYS_OFFLINE);
    
    if(pc_rx_Task_Handle != NULL)//避免任务没来得及创建就发送信号量，导致卡在断言机制中
    {
      xTaskNotifyFromISR((TaskHandle_t) pc_rx_Task_Handle, 
                         (uint32_t) PC_UART_IDLE_SIGNAL, 
                         (eNotifyAction) eSetBits, 
                         (BaseType_t *)&xHigherPriorityTaskWoken);
      
      /*进行上下文切换*/
      if(xHigherPriorityTaskWoken != pdFALSE)
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
	}
}

