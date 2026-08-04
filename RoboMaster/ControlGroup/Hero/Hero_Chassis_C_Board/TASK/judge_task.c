#include "judge_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"

extern TaskHandle_t judge_rx_Task_Handle;

UBaseType_t judge_tx_stack_surplus;
UBaseType_t judge_rx_stack_surplus;
uint8_t DATA[113] = {"AwakeLion!!!"};//该数组用来储存机器人交互的数据，可用户自行修改
void judge_tx_task(void *parm)
{
	uint32_t judge_wake_time = osKernelSysTick();
	static uint8_t i;

  while(1)
{
			i++;
//			judgement_client_packet_pack(DATA);
			send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);//发送包函数
			if(i<5)
			{
				judgement_client_graphics_draw_pack(1);//画实时显示电容
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);//发送包函数
			}
			else if(i>=5&&i<10)
			{
				judgement_client_graphics_draw_pack(2);//画实时显示Pitch,Yaw
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);//发送包函数
			}
			else if(i>=10&&i<15)
			{
				judgement_client_graphics_draw_pack(3);//画实时显示轻触开关
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);	//发送包函数				
			}
			else if(i>=15){
				judgement_client_graphics_draw_pack(4);//画实时显示速度(已删除)
				send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);	//发送包函数		
				i = 0;	
			}

    
    judge_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    vTaskDelayUntil(&judge_wake_time, 100);
  }
}


void judge_rx_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
                        (uint32_t) JUDGE_UART_IDLE_SIGNAL, 
                        (uint32_t *)&Signal, 
                        (TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & JUDGE_UART_IDLE_SIGNAL)
			{
				USART_ClearFlag(USART1, USART_FLAG_IDLE);//清除空闲中断标志位
        		dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);//通过指针取址的方法把串口接收的数据放进FIFO
        		unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);//同样再通过指针取址的方法把FIFO里的数据拿出来放进一个数组里
     		}
    }
    judge_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
  }
}

/*USART6 中断函数*/
void USART6_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
	if(USART_GetFlagStatus(USART6,USART_FLAG_IDLE) != RESET                   //判断是否空闲总线              USART_FLAG_TC
		 && USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)                   //判断是否空闲总线中断
 	{
		USART_ReceiveData(USART6);
    	USART_ClearFlag(USART6, USART_FLAG_IDLE);//清除空闲中断标志位
    	USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    	err_detector_hook(JUDGE_SYS_OFFLINE);
    
		if(judge_rx_Task_Handle != NULL) //避免任务没来得及创建就发送信号量，导致卡在断言机制中
		{
		xTaskNotifyFromISR((TaskHandle_t) judge_rx_Task_Handle, 
							(uint32_t) JUDGE_UART_IDLE_SIGNAL,
							(eNotifyAction) eSetBits,
							(BaseType_t *)&xHigherPriorityTaskWoken);
		/*进行上下文切换*/
		if(xHigherPriorityTaskWoken != pdFALSE)
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
 	}
}


