#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "string.h"
#include "pc_rx_data.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"

UBaseType_t shoot_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

extern uint8_t shoot_ready;
extern uint8_t shoot_cmd;


shoot_t  shoot;
loader_t loader[2];
char firc_mode; //发送给云台摩擦轮速度模式

void shoot_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_SHOOT_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_SHOOT_SIGNAL)
			{
        if(shoot_mode != SHOOT_DISABLE)
        {

          fric_wheel_ctrl();					  // 启动摩擦轮
          
					shoot_bullet_handler();       //判断shoot_ready控制拨盘供弹
					
					if(shoot.fric_wheel_run)//如果水平摩擦轮是打开的
					{
						if(shoot.shoot_cmd )	//判断发射标志位
							shoot_cmd = 1;			  	  //云台垂直摩擦路控制标志位
						else shoot_cmd = 0;		
					}
          else
          {

            shoot.shoot_cmd   = 0;
            shoot.c_shoot_time=0;
						shoot_cmd = 0;
            shoot.fric_wheel_spd = 0;
						
            pid_loader_spd[0].out = 0;
						pid_loader_spd[0].iout = 0;
						 pid_loader_spd[1].out = 0;
						pid_loader_spd[1].iout = 0;
						
            loader[0].angle_ref = moto_loader[0].total_angle; //记录当前拨盘电机编码位
						loader[1].angle_ref = moto_loader[1].total_angle; //记录当前拨盘电机编码位
						

          }
          
          get_last_shoot_mode();
        }
        else
        {
          pid_loader_spd[0].out = 0;
          shoot.fric_wheel_spd = 0;
        }
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) SHOT_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
      }
    }
			
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
		
} 



void get_last_shoot_mode(void)
{
	shoot.last_para_mode = shoot.para_mode;
}

/*射击模式选择
**/
static void shoot_para_ctrl(void)
{
	firc_mode = SHOOTMAD_MODE;
}

/*摩擦轮控制*/
static void fric_wheel_ctrl(void)
{
	if (shoot.fric_wheel_run)
	{
		shoot_para_ctrl();						// 射击模式选择
	}
	else
	{
		firc_mode = SHOOTSTOP_MODE;  //射击失能
	}
}

static void shoot_bullet_handler(void)
{
	if(shoot_ready == 0) 
	{
		loader[0].spd_ref = -1100;
		loader[1].spd_ref = -1100;
	}
	
	else if(shoot_ready == 1)
	{	
	loader[0].spd_ref = 0;pid_loader_spd[0].iout = 0;pid_loader_spd[0].out = 0;
	loader[1].spd_ref = 0;pid_loader_spd[1].iout = 0;pid_loader_spd[1].out = 0;
	}//拨盘pid积分/输出清零
	
  pid_calc(&pid_loader_spd[0], moto_loader[0].speed_rpm, loader[0].spd_ref);//ID 7 front
	 pid_calc(&pid_loader_spd[1], moto_loader[1].speed_rpm, loader[1].spd_ref);//ID 8 Rear

}


void shoot_param_init(void)
{
  memset(&shoot, 0, sizeof(shoot_t));
  memset(&loader, 0, sizeof(loader_t));
	
  shoot.ctrl_mode      = SHOT_DISABLE;
	shoot.para_mode			 = SHOOTMAD_MODE;
  
}


