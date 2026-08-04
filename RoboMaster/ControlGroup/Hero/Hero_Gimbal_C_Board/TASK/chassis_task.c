#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
#include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

/*云台代码，底盘部分不需要*/
chassis_t chassis;

float chassis_pid[6] = {0};//{9, 0.01f, 10.0f, 15,0, 5};//20.5,0.1

//空任务
void chassis_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_CHASSIS_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_CHASSIS_SIGNAL)
			{

				

        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                  (uint32_t) CHASSIS_MOTOR_MSG_SIGNAL, 
                  (eNotifyAction) eSetBits, 
                  (uint32_t *)NULL );
      }
    }
    
    chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

