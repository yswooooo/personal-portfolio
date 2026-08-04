#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"
#include "judge_task.h"
#include "judge_rx_data.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "detect_task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "shoot_task.h"
#include "bsp_iwdg.h"  
UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;
uint8_t doge_twist;//小陀螺低电压标志位
extern uint8_t gimbal_follow_chassis;//发送云台，底盘模式

//全局拨杆
global_status global_mode; //当前拨杆状态
global_status last_global_mode;//上一次拨杆状态
//云台
gimbal_status gimbal_mode; //当前云台模式
gimbal_status last_gimbal_mode;//上一次云台模式
//底盘
chassis_status chassis_mode; //当前底盘模式
chassis_status last_chassis_mode; //上一次底盘模式
//发射
shoot_status shoot_mode; //当前发射模式
shoot_status last_shoot_mode; //上一次发射模式

char switch_status[2]={0};//发送模式给云台开发板
extern char gimble_action;//发送给云台判断是否有动遥杆
extern char firc_mode; //发送给云台摩擦轮速度模式

void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
  while(1)
  {
	
    get_last_mode();//获取上一次模式
    get_main_mode();//判断右拨杆现为正常模式还是自瞄模式
    get_chassis_mode();//根据正常模式/自瞄模式与相关标志位判断底盘该处于小陀螺还是分离还是跟随(该部分重要的是标志位的判断【KB、RC、Capacity】)
    get_gimbal_mode();//根据正常模式/自瞄模式与相关标志位判断云台该处于小陀螺还是分离还是跟随,当为非自瞄模式时，云台配合底盘进行模式切换
    get_shoot_mode();//非REALEASE则使能摩擦轮，同时进行自爆模式是否启用的判断

		switch_status[LEFT_SWITCH] = rc.sw1;
		switch_status[RIGHT_SWITCH] = rc.sw2;
		
	//遥控器未开启时，把各个标志位置0；机器人死亡时也置0（此处以yaw轴电机是否得电判断机器人是否死亡，由于开发板单独供电，故需这么做）
	if(global_err.list[REMOTE_CTRL_OFFLINE].err_exist == 1 || global_err.list[GIMBAL_YAW_OFFLINE].err_exist == 1)
	{
			switch_status[RIGHT_SWITCH] = RC_DN;//软件下打
			firc_mode = SHOOTSTOP_MODE; //水平摩擦轮失能
			shoot.shoot_boom_flag = 0; //自爆失能
			chassis.dodge_ctrl = 0;  //小陀螺失能
		  chassis.separt_ctrl = 0; //分离模式失能
	}
	send_servant_mode(switch_status[LEFT_SWITCH],switch_status[RIGHT_SWITCH],firc_mode,gimble_action);//发送键盘模式给云台开发板
	send_bullet_shoot_cmd();//cv按键，X倍镜开关按键
	IWDG_Feed();//喂狗
	/****************************TODO:在有电机离线时，可通过开发板上的灯判断那个电机离线***************************************/
 
	
	
	
	
	
  /*******************************************************************/
	
		
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                        (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                        (eNotifyAction) eSetBits, 
                        (uint32_t *)NULL );
    
    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	

	
    vTaskDelayUntil(&mode_switch_wake_time, 2);
  }
}

void get_last_mode(void)
{
  last_global_mode = global_mode;//获取上一次全局状态
  last_gimbal_mode = gimbal_mode;//获取上一次云台状态
  last_chassis_mode = chassis_mode;//获取上一次底盘状态
  last_shoot_mode = shoot_mode;//获取上一次射击状态
}
/*********************原理是利用遥控器拨杆传递数据，从而进行机器人的模式切换*********************/
//全局
void get_main_mode(void)
{
	if(global_err.list[REMOTE_CTRL_OFFLINE].err_exist == 1 || global_err.list[GIMBAL_YAW_OFFLINE].err_exist == 1)//遥控离线或YAW电机掉电
	{
      global_mode = RELEASE_CTRL;//!
	}		
	
	else
	{
  switch(rc.sw2)
  {
    case RC_UP: //右拨杆上打时云台为正常模式(跟随)，除非鼠标右键长按强制云台进入自瞄
    {
      if(TRACK_CTRL)//"鼠标"右键长按进入自瞄模式
			{
        global_mode = SEMI_AUTOMATIC_CTRL;
				gimbal_follow_chassis = GIMBAL_FOLLOW_TRACE;//强制云台进入自瞄
			}
      else
			{
        global_mode = MANUAL_CTRL; //正常模式
				gimbal_follow_chassis = GIMBAL_FOLLOW_NORMAL;//云台正常跟随
			}
    }
    break;
		
    case RC_MI: //右拨杆中打
      global_mode = SEMI_AUTOMATIC_CTRL;//遥控器进入自瞄模式
    break;
		
    case RC_DN://右拨杆现实下打
    {
      global_mode = RELEASE_CTRL;//机器人无力
    }
    break;
		
    default:
    {
    }break;
  }
	}
}

//底盘
void get_chassis_mode(void)
{
	static uint8_t dodge_cnt,seperate_cnt;//键盘软件延时变量

	/* 小陀螺标志位 */
	if(chassis.CapData[1] < 14||judge_recv_mesg.power_heat_data.buffer_energy<10)//电容电压低或者缓冲低的时候关闭小陀螺
		doge_twist = OFF;
	if(chassis.CapData[1] > 16)
		doge_twist = ON;

	/* 小陀螺标志位解码 */
  if(KB_DODGE_CTRL && doge_twist == ON && dodge_cnt++>60) //小陀螺模式E 或者遥控器滚轮 i++——按键延时，使shift加按键更容易退出
	{ 
    chassis.dodge_ctrl = ON; //小陀螺开关置1
    chassis.separt_ctrl = OFF;//分离模式开关置0
  } 

  if(KB_DODGE_CTRL_CLOSE || doge_twist != ON) //若KB小陀螺关闭，且电压不足，小陀螺开关置0
	{ 
    chassis.dodge_ctrl = OFF;  //小陀螺模式开关置0
    dodge_cnt = 0;   //重置计数值
  }

	/* 分离模式标志位解码 */
	if(KB_SEPART_OPEN && seperate_cnt++>60) //分离模式R i++——按键延时，使shift加按键更容易退出
	{	
    chassis.separt_ctrl = ON; //分离模式开关置1
    chassis.dodge_ctrl = OFF; //小陀螺模式开关置0
   } 
	if(KB_SEPART_CLOSE)
	{	
    chassis.separt_ctrl = OFF; //分离模式开关置0
    seperate_cnt = 0; //重置计数值
  }
  //!!!
  switch(global_mode)
  {

    case MANUAL_CTRL: //正常
    {
      if((RC_DODGE_MODE || chassis.dodge_ctrl) && doge_twist == ON) //如果开启躲避模式，底盘进入小陀螺
        chassis_mode = CHASSIS_DODGE_MODE; //小陀螺模式
			else if(chassis.separt_ctrl == ON) 
				chassis_mode = CHASSIS_SEPARATE_MODE;//分离模式
      else                                   
        chassis_mode = CHASSIS_NORMAL_MODE; //底盘云台正常模式(跟随)

    }break;
    
    case SEMI_AUTOMATIC_CTRL:   // 自瞄
    {
      if((RC_DODGE_MODE || chassis.dodge_ctrl) && doge_twist == ON)   //如果进入自瞄，且开启小陀螺，整车边自瞄边小陀螺
        chassis_mode = CHASSIS_DODGE_MODE;
			else if(chassis.separt_ctrl == ON)
				chassis_mode = CHASSIS_SEPARATE_MODE; //如果进入自瞄，且开启分离模式，底盘进入分离
      else                         
        chassis_mode = CHASSIS_STOP_MODE; //底盘静止
    }break;

		case RELEASE_CTRL: //关闭遥控器
    {
			chassis_mode = CHASSIS_RELEASE;
			for (int i = 0; i < 4; i++)
			{
				glb_cur.chassis_cur[i] = pid_calc(&pid_chassis_vx_vy_spd[i], chassis.wheel_spd_fdb[i], 0); //机器人底盘无力
			}
		}break;
    
    default:
    {
    }break;
  }
}
//云台
void get_gimbal_mode(void)
{  
  switch(global_mode)
  {
    case MANUAL_CTRL://正常
    {
      if(chassis_mode == CHASSIS_DODGE_MODE)
			{
        gimbal_mode = GIMBAL_DODGE_MODE; 
				gimbal_follow_chassis = GIMBAL_FOLLOW_DODGE;//小陀螺
			}
			else if(chassis_mode == CHASSIS_SEPARATE_MODE)
			{
				gimbal_mode = GIMBAL_SEPARATE_MODE;
				gimbal_follow_chassis = GIMBAL_FOLLOW_SEPERATE;//分离
			}
      else
        gimbal_mode = GIMBAL_NORMAL_MODE; //云台底盘跟随模式
    }
    break;
    
    case SEMI_AUTOMATIC_CTRL://自瞄
    {
        gimbal_follow_chassis = GIMBAL_FOLLOW_TRACE;
    }
    break;
     
    case RELEASE_CTRL:
    {
      gimbal_mode = GIMBAL_RELEASE;
      gimbal.state = GIMBAL_INIT_NEVER;//遥控显示下打归中预备标志位，告知云台你启动时首先要进行归中
    }break;
    
  }

}
//发射
void get_shoot_mode(void)
{
	static uint8_t shootboom_cnt;
	//static uint8_t image_cnt;
  switch(global_mode)
  {
    case MANUAL_CTRL:
    case SEMI_AUTOMATIC_CTRL:
    {
      shoot_mode = SHOOT_ENABLE; //除关闭模式外，使能发射
			if(KB_SHOOT_BOOM && shootboom_cnt++ > 60) shoot.shoot_boom_flag = ON; //B键自爆模式
			if(KB_SHOOT_BOOM_STOP){shootboom_cnt = 0; shoot.shoot_boom_flag = OFF;}
//			
//			if(KB_IMAGE && image_cnt++ > 60) shoot.shoot_image_flag = ON; //F键图传角度改变
//			if(KB_IMAGE_STOP){image_cnt = 0; shoot.shoot_image_flag = OFF;}
    }break;
    
    case RELEASE_CTRL:
      shoot_mode = SHOOT_DISABLE;//发射失能
    break;
    
    default:
    {
    }break;
  }
}
