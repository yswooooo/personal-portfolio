#include "modeswitch_task.h"

#include "STM32_TIM_BASE.h"
#include "detect_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "shoot_task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "bsp_can.h"
#include "math.h"
#include "arm_math.h"
#include "pc_rx_data.h"
#include "bsp_iwdg.h"  

UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;
extern uint8_t gimbal_follow_chassis;//云台跟随底盘模式运动枚举
extern uint8_t shoot_ready; //拨盘运动标志位

global_status global_mode; //全局模式
global_status last_global_mode;

gimbal_status gimbal_mode; //云台模式
gimbal_status last_gimbal_mode;

chassis_status chassis_mode; //底盘模式
chassis_status last_chassis_mode;

shoot_status shoot_mode; //发射模式
shoot_status last_shoot_mode;

void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
  while(1)
  {
    get_main_mode();
    get_gimbal_mode();
	
		//发送pit的相对角度和视觉有效位以及轻触开关是否触碰到的标志位
		send_to_chassis(gimbal.sensor.pit_relative_angle,pc_recv_mesg.mode_Union.info.visual_valid,shoot_ready);
	
		/**************************************************************************/
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                  (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                  (eNotifyAction) eSetBits, 
                  (uint32_t *)NULL );

	IWDG_Feed();//喂狗
	  
    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    vTaskDelayUntil(&mode_switch_wake_time, 2);
  }
}
/*获取上一次的所有模式，包括全局、云台、底盘、发射机构*/
void get_last_mode(void)
{
  last_global_mode = global_mode;//获取上一次全局状态
  last_gimbal_mode = gimbal_mode;//获取上一次云台状态
  last_chassis_mode = chassis_mode;//获取上一次底盘状态
  last_shoot_mode = shoot_mode;//获取上一次射击状态
}
/*通过底盘传输上来的拨杆数据`rc`、gimbal_follow_chassis进行模式选择*/
void get_main_mode(void)
{   
  switch(rc.sw2)//遥控接收器插在底盘，云台通过CAN2->CAN1通信软件模拟RC遥控
  {
    case RC_UP: //遥控右拨杆打上
    {
      if(gimbal_follow_chassis == GIMBAL_FOLLOW_TRACE)//鼠标右键进入视觉
        global_mode = SEMI_AUTOMATIC_CTRL;
      else
        global_mode = MANUAL_CTRL;
    }
    break;
    case RC_MI: //遥控右拨杆打中
      global_mode = SEMI_AUTOMATIC_CTRL;
    break;
    case RC_DN://①机器人阵亡、pitch/yaw电机离线、遥控离线；②遥控右拨杆现实下打
    {
      global_mode = RELEASE_CTRL;//引发云台预备归中
      ramp_init(&pit_ramp, 300);
      ramp_init(&yaw_ramp, 500);
    }
    break;
    default:
    {
    }break;
  }
}
/*根据`global_mode`进行云台模式选择*/
void get_gimbal_mode(void)
{  
  switch(global_mode)
  {
    case MANUAL_CTRL://正常
    {
			if(gimbal.state == GIMBAL_INIT_NEVER)//当底盘传上来的 gimbal.state 标志位 为 GIMBAL_INIT_NEVER 时，云台首先进行归中
			{
				gimbal_mode = GIMBAL_INIT;//云台归中
			}
			else //归中完毕后才执行配合底盘的操作
			{
      
      if(gimbal_follow_chassis == GIMBAL_FOLLOW_DODGE)
        gimbal_mode = GIMBAL_DODGE_MODE; //小陀螺模式
			else if(gimbal_follow_chassis == GIMBAL_FOLLOW_SEPERATE)
				gimbal_mode = GIMBAL_SEPARATE_MODE; //分离模式
      else
        gimbal_mode = GIMBAL_NORMAL_MODE; //云台底盘跟随模式
			}
    }
    break;
    
    case SEMI_AUTOMATIC_CTRL: //自瞄
    {
        gimbal_mode = GIMBAL_TRACK_ARMOR; //云台自瞄模式
    }
    break;
    
    case RELEASE_CTRL:
    {
      gimbal_mode = GIMBAL_RELEASE;//在gimbal_task.c中的 if 判断会执行 gimbal.state = GIMBAL_INIT_NEVER;
      gimbal.state = GIMBAL_INIT_NEVER;//云台预备归中(NOTE:这句话要加上去，不然云台有时候不会归中)

    }break;
    
  }
}
