#include "info_get_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "keyboard.h"
#include "remote_ctrl.h"
#include "bsp_can.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "imu_task.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "stdlib.h"
#include "math.h"

UBaseType_t info_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t gimbal_Task_Handle;

extern TaskHandle_t shoot_Task_Handle;


void info_get_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if(Signal & MODE_SWITCH_INFO_SIGNAL)
			{
        taskENTER_CRITICAL();
       

        get_gimbal_info();

        get_global_last_info();
				

				
        taskEXIT_CRITICAL();
        if(global_mode == RELEASE_CTRL)
        {
          MEMSET(&glb_cur,motor_current_t);
          xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                              (uint32_t) MODE_SWITCH_MSG_SIGNAL, 
                              (eNotifyAction) eSetBits, 
                              (uint32_t *)NULL );
        } 
        else
        {
          xTaskGenericNotify( (TaskHandle_t) gimbal_Task_Handle, 
                            (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );

          xTaskGenericNotify( (TaskHandle_t) shoot_Task_Handle, 
                            (uint32_t) INFO_GET_SHOOT_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
        }
        
        info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      }
    }

  }
}

static void get_gimbal_info(void)
{
  /* 转换成相对角度 */
  static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR*YAW_DECELE_RATIO/ENCODER_ANGLE_RATIO;
  static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR*PIT_DECELE_RATIO/ENCODER_ANGLE_RATIO;
	
  gimbal.sensor.yaw_relative_angle = yaw_ecd_ratio*get_relative_pos(moto_yaw.ecd, gimbal.yaw_center_offset);
  gimbal.sensor.pit_relative_angle = -pit_ecd_ratio*get_relative_pos(moto_pit.ecd, gimbal.pit_center_offset);

}

static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)						//分中轴线象限
  {
		if (raw_ecd - center_offset > - 4096)
      tmp = raw_ecd - center_offset;							//在右边 = 负数
    else
      tmp = raw_ecd + 8192 - center_offset;				//在左边 = 正数
  }
  else
  {
		if (raw_ecd - center_offset > 4096)
      tmp = raw_ecd - 8192 - center_offset;				//在右边 = 负数
    else
      tmp = raw_ecd - center_offset;							//在左边 = 正数
  }
  return tmp;
}

/**
*函数名：yaw_180_to_900
*作用：将相对角度的范围从 -180~189 扩大到 -900~900（该函数可以最大到10圈，但一般只用3圈就够了）
*参数：无
*返回值：相对角扩大后的角度
**/
float yaw_encode;
signed char i_temp = 0;//用于记录yaw轴转了多少圈
float yaw_180_to_900(void)
{
	static float last_yaw;
	
	if(last_yaw > 158 && gimbal.sensor.yaw_relative_angle < -158)
		i_temp++;															//圈数加一
	else if(last_yaw < -158 && gimbal.sensor.yaw_relative_angle > 158)
		i_temp--;															//圈数减一
	if(i_temp>10)i_temp = -10;      if(i_temp<-10)i_temp = 10; //限定范围10圈
	yaw_encode = i_temp*360*1.0 + gimbal.sensor.yaw_relative_angle; //解算后扩大的角度
	
	last_yaw = gimbal.sensor.yaw_relative_angle;
	
  return yaw_encode;
}

//获取上一次的拨杆值
static void get_global_last_info(void)
{
  glb_sw.last_sw1 = rc.sw1;
  glb_sw.last_sw2 = rc.sw2;  
}

