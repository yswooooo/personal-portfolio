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
#include "imu_task.h" //！
#include "sys_config.h"
#include "pc_rx_data.h"
#include "stdlib.h"
#include "math.h"

UBaseType_t info_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t gimbal_Task_Handle;
extern TaskHandle_t chassis_Task_Handle;
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
        
        keyboard_global_hook();//获取鼠标状态值（mouse）
        get_chassis_info(); //获取底盘控制(rc/kb)
        get_gimbal_info(); //获取云台控制(rc/kb)
        get_shoot_info(); //获取发射控制(rc/kb)
        get_global_last_info();//获取上一次拨杆值last.rc.sw
        
        taskEXIT_CRITICAL();
        if(global_mode == RELEASE_CTRL) //当机器人为急停模式
        {
          MEMSET(&glb_cur,motor_current_t); //底盘电机电流给零
          xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                              (uint32_t) MODE_SWITCH_MSG_SIGNAL, 
                              (eNotifyAction) eSetBits, 
                              (uint32_t *)NULL );
        } 
        else
        {
          xTaskGenericNotify( (TaskHandle_t) gimbal_Task_Handle, //通知云台
                            (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
          xTaskGenericNotify( (TaskHandle_t) chassis_Task_Handle, //通知底盘
                            (uint32_t) INFO_GET_CHASSIS_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
          xTaskGenericNotify( (TaskHandle_t) shoot_Task_Handle, //通知发射机构
                            (uint32_t) INFO_GET_SHOOT_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
        }
        
        info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      }
    }

  }
}
//底盘信息获取与控制信息获取
static void get_chassis_info(void)
{
  /* 得到底盘轮速 */
  for (uint8_t i = 0; i < 4; i++)
  {
    chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
  }
  /* 获取底盘遥控和键鼠的数据 */
  keyboard_chassis_hook();
  remote_ctrl_chassis_hook();
	
}
//云台信息获取与控制信息获取
static void get_gimbal_info(void)
{
  /* 转换成相对角度 */
  static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR*YAW_DECELE_RATIO/ENCODER_ANGLE_RATIO;
  gimbal.sensor.yaw_relative_angle = yaw_ecd_ratio*get_relative_pos(moto_yaw.ecd, gimbal.yaw_center_offset);

  /*获取云台遥控和键鼠的数据*/
  keyboard_gimbal_hook();
  remote_ctrl_gimbal_hook();
  
}
//发射机构信息获取与控制信息获取
static void get_shoot_info(void)
{
   /*获取发射遥控和键鼠的数据*/
  remote_ctrl_shoot_hook();
  keyboard_shoot_hook();
  
}
//获取上一次遥控拨杆状态
static void get_global_last_info(void)
{
  glb_sw.last_sw1 = rc.sw1;
  glb_sw.last_sw2 = rc.sw2;  
}

//获取pitch/yaw相对角
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)  //电机转一圈是8192，半圈是4096 (0~8192)
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

