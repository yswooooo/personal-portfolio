#include "keyboard.h"
#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "sys_config.h"
#include "ramp.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "judge_rx_data.h"//!!!
#include "ladrc.h"
kb_ctrl_t km;//键鼠控制变量

uint8_t left_offset_cmd;//扩展装甲板(左)
uint8_t right_offset_cmd;//扩展装甲板(右)

void get_mouse_status(MOUSE_STATUS *status,uint8_t mouse)
{
  switch(*status)
  {
    case MOUSE_RELEASE:
    {
      if(mouse)
        *status = MOUSE_PRESS;
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_PRESS:
    {
      if(mouse)
        *status = MOUSE_DONE;
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_DONE:
    {
      if(mouse)
      {
        *status = MOUSE_ONCE;
        if(status == &km.l_mouse_sta)
          km.l_cnt = HAL_GetTick();
        else
          km.r_cnt = HAL_GetTick();
      }
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_ONCE:
    {      
      if(mouse)
      {
        if(status == &km.l_mouse_sta)
        {
          if(HAL_GetTick() - km.l_cnt > 100)
            *status = MOUSE_LONG;
        }
        else
        {
          if(HAL_GetTick() - km.r_cnt > 100)
            *status = MOUSE_LONG;
        }
      }
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_LONG:
    {
      if(!mouse)
      {
        *status = MOUSE_RELEASE;
      }
    }break;
    
    default:
    {
    }break;
  }
}

void keyboard_global_hook(void)
{
  if (km.kb_enable)
  {
    get_mouse_status(&km.l_mouse_sta, rc.mouse.l);
    get_mouse_status(&km.r_mouse_sta, rc.mouse.r);
  }
}


/*控制方向*/
static void chassis_direction_ctrl(uint8_t forward, uint8_t back,
                                uint8_t left,    uint8_t right)
{
  //前后控制(前正后负)
  if(back)
  {
    km.vx = -1;
		rm.vx=0;
		rm.vy=0;
		rm.vw=0;
  }
  else if(forward)
  {
    km.vx = 1;
		rm.vx=0;
		rm.vy=0;
		rm.vw=0;
  }
  else
  {
   km.vx = 0;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
  
  //左右控制(左正右负)
  if(right)
  {
   km.vy =  -1 ;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
  else if(left)
  {
   km.vy = 1;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
  else
  {
   km.vy = 0;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
}


/*控制摩擦轮*/
static void firc_ctrl(uint8_t firc_open,uint8_t firc_close)
{
  if(firc_open)
  {
    shoot.fric_wheel_run = 1;
  }
  if(firc_close)
  {
    shoot.fric_wheel_run = 0;
  }
}

#define Jude_or_not 1

/*控制单发和连发命令*/
static void shoot_cmd_ctrl(uint8_t shoot_cmd,uint8_t c_shoot_cmd)
{
	//若裁判系统发送42mm弹丸数据
	if(judge_recv_mesg.power_heat_data.shooter_42mm_barrel_heat==100)
	{
		if(shoot_cmd  
			&&(judge_recv_mesg.power_heat_data.shooter_42mm_barrel_heat <= judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - 110 || shoot.shoot_boom_flag == 1)
			&&(HAL_GetTick() - shoot.c_shoot_time>=800 || shoot.shoot_boom_flag == 1))  //延时800ms防止点击过快超热量，开启自爆模式不限热量
		{
			shoot.c_shoot_time = HAL_GetTick();
			shoot.shoot_cmd = 1;
		}	
	}
   //else逻辑与if逻辑相同,其差别在于是否有800ms延时
	//else逻辑用于裁判系统未发数据情况
	else
	{
		if(shoot_cmd  
			&&(judge_recv_mesg.power_heat_data.shooter_42mm_barrel_heat <= judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - 100 || shoot.shoot_boom_flag == 1))//热量限制>100不进行延时，提高射击频率
		{
			shoot.c_shoot_time = HAL_GetTick();
			shoot.shoot_cmd = 1;
		}	
	
	}
	if(Jude_or_not)
	{
		if(judge_recv_mesg.game_robot_state.power_management_shooter_output==0)//防止死亡后误点鼠标重新上电后自动发射一颗弹丸
		{                                                                 //若发射机构断电，则发射失能
			shoot.shoot_cmd = 0;
			shoot.c_shoot_time=0;
		}
	}
	/* 键鼠部分发弹防连发保险判断 ，发弹后超过70ms后强制停发 */   
  if(HAL_GetTick() - shoot.c_shoot_time >=70)    
  {                                                              
    shoot.shoot_cmd = 0;
  }
}
static uint8_t  KM_LADRC  = 1;
LADRC_NUM kb_pit_ref = 
{
   .r = 30,     //速度因子
   .h = 0.002,            //积分步长
};
LADRC_NUM kb_yaw_ref = 
{
   .r = 35,     //速度因子
   .h = 0.002,            //积分步长
};

/*鼠标控制云台灵敏度*/
static void gimbal_speed_ctrl(int16_t pit_ref_spd, int16_t yaw_ref_spd)
{
	if(KM_LADRC)
	{
		/* TD跟踪微分处理 */
		LADRC_TD(&kb_pit_ref, pit_ref_spd * 0.005f * GIMBAL_PC_MOVE_RATIO_PIT );
		LADRC_TD(&kb_yaw_ref, yaw_ref_spd * 0.005f * GIMBAL_PC_MOVE_RATIO_YAW );
		
		km.pit_v = kb_pit_ref.v1;
		km.yaw_v = kb_yaw_ref.v1;

	}
	else
	{
		  km.pit_v = pit_ref_spd *0.005f  * GIMBAL_PC_MOVE_RATIO_PIT;
		km.yaw_v = yaw_ref_spd * 0.005f  * GIMBAL_PC_MOVE_RATIO_YAW;
	}

}

/*自瞄添加CV按键，发送标志位给给算法做扩展装甲板*/
static void gimbal_track_cvcmd(uint8_t left_offset,uint8_t right_offset)
{
	if(left_offset)
		left_offset_cmd=1;
	else
		left_offset_cmd=0;
	if(right_offset)
		right_offset_cmd=1;
	else
		right_offset_cmd=0;
	
}
/*倍镜键盘按键发送给云台*/
static void shoot_glass_ctrl(uint8_t glass_open,uint8_t glass_close)
{
	if(glass_open)
		shoot.shoot_glass_flag = 1;
	
	if(glass_close)
		shoot.shoot_glass_flag = 0;

}
/*图传键盘按键发送给云台*/
static void shoot_image_ctrl(uint8_t image_normal,uint8_t image_down)
{
	if(image_down)
		shoot.shoot_image_flag = 1;
	
	if(image_normal)
		shoot.shoot_image_flag = 0;

}
void keyboard_chassis_hook(void)
{
  if(km.kb_enable)
  {
    chassis_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
  }
  else
  {
    km.vx = 0;
    km.vy = 0;
  }
}

void keyboard_shoot_hook(void)
{
  if(km.kb_enable)
  {
    firc_ctrl(KB_OPEN_FRIC_WHEEL, KB_CLOSE_FIRC_WHEEL);//摩擦轮
    shoot_cmd_ctrl(KB_SINGLE_SHOOT, KB_CONTINUE_SHOOT);//发弹
		shoot_glass_ctrl(KB_GLASS_CTRL,KB_GLASS_CLOSE);//倍镜
	  shoot_image_ctrl(KB_IMAGE_STOP,KB_IMAGE);//图传
  }
}

void keyboard_gimbal_hook(void)
{
  if (km.kb_enable)
  {
    gimbal_speed_ctrl(rc.mouse.y, rc.mouse.x);
		gimbal_track_cvcmd(LEFT_OFFSET,RIGHT_OFFSET);
  }
  else
  {
    km.pit_v = 0;
    km.yaw_v = 0;
    gimbal.track_ctrl = 0;
		left_offset_cmd=0;
		right_offset_cmd=0;
  }
}
