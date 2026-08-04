#include "comm_task.h"
#include "STM32_TIM_BASE.h"
#include "gimbal_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "judge_task.h"
#include "judge_rx_data.h"
#include "chassis_task.h"
#include "keyboard.h"

UBaseType_t can_stack_surplus;
motor_current_t glb_cur;
extern pid_t pid_chassis_power_buffer;

void can_msg_send_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {

    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL  | \
															CHASSIS_MOTOR_MSG_SIGNAL | \
															SHOT_MOTOR_MSG_SIGNAL  | \
															MODE_SWITCH_MSG_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
	if(STAUS == pdTRUE)
	{
		if(Signal & GIMBAL_MOTOR_MSG_SIGNAL)//发送云台角度
		{
			send_servant_angle(gimbal_angle[GIMBAL_YAW_ANGLE],gimbal_angle[GIMBAL_PITCH_ANGLE]); //can2发送
		}
			
		if(Signal & CHASSIS_MOTOR_MSG_SIGNAL)//发送底盘电流
		{
			send_chassis_cur(glb_cur.chassis_cur[0],glb_cur.chassis_cur[1],glb_cur.chassis_cur[2],glb_cur.chassis_cur[3]); //!!!
		}
			
		if(Signal & SHOT_MOTOR_MSG_SIGNAL) //发送拨盘电机
		{
			send_trig_cur1(pid_loader_spd[0].out,pid_loader_spd[1].out);
		}
		if(chassis_mode == CHASSIS_RELEASE)
		{
			send_chassis_cur(glb_cur.chassis_cur[0],glb_cur.chassis_cur[1],glb_cur.chassis_cur[2],glb_cur.chassis_cur[3]); //!!!
		}

		if(Signal & MODE_SWITCH_MSG_SIGNAL) //发送电流全为0
		{
			send_chassis_cur(0,0,0,0);
			send_trig_cur1(0,0);
		}
	}
    can_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

  }
}
