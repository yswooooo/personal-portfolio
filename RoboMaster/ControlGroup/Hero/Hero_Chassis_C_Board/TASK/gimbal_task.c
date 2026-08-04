#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "chassis_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()函数
#include "math.h"   //fabs()函数


UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

float gimbal_angle[2] = {0};//发送角度给另一块开发板
char gimble_action = 0;//发送给另一块板yaw是否运动

float loader_pid[6] = {0,0,0,10,0.05,0};//拨盘pid参数


gimbal_t gimbal;//云台控制结构体
void gimbal_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  while(1)
	{
		STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
	 {
	 if(Signal & INFO_GET_GIMBAL_SIGNAL)
		{
			/* 拨盘 电机的PID参数 */
			for(int i = 0; i < 2; i++)
			{
				PID_Struct_Init(&pid_loader_angle[i], loader_pid[0], loader_pid[1], loader_pid[2], 4000, 300, DONE);
				PID_Struct_Init(&pid_loader_spd[i], loader_pid[3], loader_pid[4], loader_pid[5],16000, 10000,DONE);
			}
        
        if(gimbal_mode != GIMBAL_RELEASE)
        {
          switch(gimbal_mode)
          {
            /*云台底盘跟随模式*/
            case GIMBAL_NORMAL_MODE:
            {
              nomarl_handler();
            }break;
			case GIMBAL_TRACK_ARMOR: //自瞄与跟随都用同一个正常模式
			{
				nomarl_handler();
			}
            /*云台底盘分离模式*/
            case GIMBAL_SEPARATE_MODE:
            {
              separate_handler();
            }break;
            /*小陀螺模式*/
            case GIMBAL_DODGE_MODE:
            {
              dodge_handler();
            }break;
            
            default:
            {
            }break;
          }
        }

        last_gimbal_mode = gimbal_mode;//获取上一次云台状态
				
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                            (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
      }
     }
    
    gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

	}
  
}
/*云台初始化*/
void gimbal_param_init(void)
{
	/* 拨盘 电机的PID参数 */
	for(int i = 0; i < 2; i++)
	{
		//PID_Struct_Init(&pid_loader_angle[i], loader_pid[0], loader_pid[1], loader_pid[2], 4000, 300, INIT);
		PID_Struct_Init(&pid_loader_spd[i], loader_pid[3], loader_pid[4], loader_pid[5],16500, 4000,INIT);
	}   
}

/*判断yaw轴是否有输入*/
static gimbal_state_t remote_is_action(void)
{
  if ((abs(rc.ch3) >= 10) || (abs(rc.mouse.x) >= 1))
  {
    return IS_ACTION;
  }
  else
  {
      return NO_ACTION;
  }
}

//自瞄与跟随都用同一个正常模式
uint32_t no_action_time;
uint32_t debug_time = 1000;
static void nomarl_handler(void)
{ 
	if(gimbal_mode != GIMBAL_TRACK_ARMOR)
	{
		if(last_gimbal_mode != GIMBAL_NORMAL_MODE) //如果上一次不是正常模式
		{
			no_action_time = HAL_GetTick();          //记录时间
			chassis.chassis_flow = 1;                //底盘归中跟随
		}
		else  
		{
			gimbal.state = remote_is_action(); //判断yaw轴是否有输入
			gimble_action = gimbal.state;
			//这是一个中介点
			if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
			{			
				no_action_time = HAL_GetTick();   //为下一个else if 服务
				gimbal.pid.yaw_angle_ref = 0 ;
			}
			else if((gimbal.last_state == NO_ACTION) //若上一次和这一次都没动yaw遥控
						&&(gimbal.state == NO_ACTION)      //且没动遥控间隔时间大于设定值
						&&(HAL_GetTick() - no_action_time > debug_time)) //延时等待底盘归中完毕
			{
				chassis.chassis_flow = 0; //底盘不跟随
				gimbal.pid.yaw_angle_ref = 0; //遥控器无输入时，给定要清零
			}
			//只有在这一次和上一次yaw轴有输入的时候才会给yaw给定
			else if(gimbal.last_state == IS_ACTION && gimbal.state == IS_ACTION)
			{
				gimbal.pid.yaw_angle_ref = ((-rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW)
																					- km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW)*0.65f;//给定 = 遥控器给定+键盘给定 ，遥控器正负与方向有关
				chassis.chassis_flow = 1; //遥控器有输入时，底盘跟随云台
			}
		}
		gimbal_angle[GIMBAL_YAW_ANGLE] = gimbal.pid.yaw_angle_ref;//gimbal_angle[]用于存放要发给云台开发板数据
	}
	
	//自瞄模式时，底盘一直跟随云台（此处有bug。bug描述：自瞄模式下，底盘还是不会跟随云台的，及本行代码没起作用，但不影响正常运动）
	else if(gimbal_mode == GIMBAL_TRACK_ARMOR)
		chassis.chassis_flow = 1;

  /*pitch轴*/
  	gimbal.pid.pit_angle_ref = (-rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT) 
                         + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;//给定 = 遥控器给定+键盘给定 遥控器正负与方向有关
	
	gimbal_angle[GIMBAL_PITCH_ANGLE] = gimbal.pid.pit_angle_ref; //gimbal_angle[]用于存放要发给云台开发板数据
	
	gimbal.last_state = remote_is_action();//获取上次yaw轴遥控器输入的状态	
}

static void separate_handler(void)
{
		chassis.chassis_flow = 0; //底盘不跟随、
	
		gimbal.state = remote_is_action(); //判断yaw轴是否有输入
		gimble_action = gimbal.state;
	
		if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
		{			
			no_action_time = HAL_GetTick();
		}
		else if((gimbal.last_state == NO_ACTION) 
					&&(gimbal.state == NO_ACTION)
					&&(HAL_GetTick() - no_action_time > 50))
		{
			gimbal.pid.yaw_angle_ref = 0; //遥控器无输入时要把给定清零（云台开发板给定值是累加的，不清零会被一直累加）
		}
		else if(gimbal.last_state == IS_ACTION && gimbal.state == IS_ACTION)
		{
			gimbal.pid.yaw_angle_ref = (-rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW)
																- km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW; //给定 = 遥控器给定+键盘给定 遥控器正负与方向有关
		}

  /*pitch轴*/
  gimbal.pid.pit_angle_ref = (-rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT) 
                         + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT; //给定 = 遥控器给定+键盘给定 遥控器正负与方向有关
		
	gimbal_angle[GIMBAL_PITCH_ANGLE] = gimbal.pid.pit_angle_ref; //gimbal_angle[]用于存放要发给云台开发板数据
	gimbal_angle[GIMBAL_YAW_ANGLE] = gimbal.pid.yaw_angle_ref;	
		
	gimbal.last_state = remote_is_action();//获取上次输入的状态
}

static void dodge_handler(void)
{ 
	gimbal.state = remote_is_action(); //判断yaw轴是否有输入
	gimble_action = gimbal.state;
	if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
	{			
		no_action_time = HAL_GetTick();
		gimbal.pid.yaw_angle_ref = 0;
	}
	else if((gimbal.last_state == NO_ACTION) 
				&&(gimbal.state == NO_ACTION)
				&&(HAL_GetTick() - no_action_time > debug_time))
	{
		gimbal.pid.yaw_angle_ref = 0; //遥控器无输入时，给定要清零
	}
	else if(gimbal.last_state == IS_ACTION && gimbal.state == IS_ACTION)
	{
		gimbal.pid.yaw_angle_ref = (-rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW)
															- km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW;//给定 = 遥控器给定+键盘给定 遥控器正负与方向有关
		gimbal_angle[GIMBAL_YAW_ANGLE] = gimbal.pid.yaw_angle_ref;//gimbal_angle[]用于存放要发给云台开发板数据
	}

	/*pitch轴*/
	gimbal.pid.pit_angle_ref = (-rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT) 
												 + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;//给定 = 遥控器给定+键盘给定 遥控器正负与方向有关
	gimbal_angle[GIMBAL_PITCH_ANGLE] = gimbal.pid.pit_angle_ref;//gimbal_angle[]用于存放要发给云台开发板数据
		
	gimbal.last_state = remote_is_action();//获取上次输入的状态
}  
