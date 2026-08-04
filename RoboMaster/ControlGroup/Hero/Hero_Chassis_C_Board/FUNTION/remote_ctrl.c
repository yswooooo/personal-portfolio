#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "shoot_task.h"
#include "keyboard.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"


rc_ctrl_t rm;//遥控控制变量
sw_record_t glb_sw;//记录上一次拨杆值
uint8_t num = 0;//简单的RC滤波变量
static void kb_enable_hook(void);

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf)
{
	rc->ch1 = (dbus_buf[0] | dbus_buf[1] << 8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (dbus_buf[1] >> 3 | dbus_buf[2] << 5) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (dbus_buf[2] >> 6 | dbus_buf[3] << 2 | dbus_buf[4] << 10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (dbus_buf[4] >> 1 | dbus_buf[5] << 7) & 0x07FF;
	rc->ch4 -= 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
  rc->sw1 = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (dbus_buf[5] >> 4) & 0x0003;
  rc->iw = (dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF;
	
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }
	if(((dbus_buf[6] | (dbus_buf[7] << 8))!=0 || num == 30))
	{
		rc->mouse.x = dbus_buf[6] | (dbus_buf[7] << 8); // x axis
		rc->mouse.y = dbus_buf[8] | (dbus_buf[9] << 8);
		rc->mouse.z = dbus_buf[10] | (dbus_buf[11] << 8);
	}
	else
	{
		num += 1;
	}
  rc->mouse.l = dbus_buf[12];
  rc->mouse.r = dbus_buf[13];
  rc->kb.key_code = dbus_buf[14] | dbus_buf[15] << 8; // key borad code
}
//------------------------------------------------------------------------------------------------------------------------//
/*
* @ RC_RESOLUTION :摇杆最大值 660 
* @ CHASSIS_RC_MAX_SPEED_X
    CHASSIS_RC_MAX_SPEED_Y
    CHASSIS_RC_MAX_SPEED_R ：平移和旋转的速度最大值
  @ CHASSIS_RC_MOVE_RATIO_X
    CHASSIS_RC_MOVE_RATIO_Y
    CHASSIS_RC_MOVE_RATIO_R : 数值方向
*/
static void chassis_operation_func(int16_t forward_back, int16_t left_right)
{
  rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X * CHASSIS_RC_MOVE_RATIO_X; //右↑ y
  rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y * CHASSIS_RC_MOVE_RATIO_Y;  // 右← x

}

void remote_ctrl_chassis_hook(void)
{
  chassis_operation_func(rc.ch2, rc.ch1);
}
//------------------------------------------------------------------------------------------------------------------------//
static void gimbal_operation_func(int16_t pit_ctrl, int16_t yaw_ctrl)
{
  rm.pit_v = pit_ctrl * 0.0007f * GIMBAL_RC_MOVE_RATIO_PIT * 0.3f ; //↑ -
  rm.yaw_v = -yaw_ctrl * 0.0007f * GIMBAL_RC_MOVE_RATIO_PIT * 0.3f; //← +
}

void remote_ctrl_gimbal_hook(void)
{
  gimbal_operation_func(rc.ch4, rc.ch3);
}
//------------------------------------------------------------------------------------------------------------------------//
static void rc_fric_ctrl(uint8_t ctrl_fric)
{
  if (ctrl_fric)
  {
    shoot.fric_wheel_run = !shoot.fric_wheel_run;//开摩擦轮
		shoot.shoot_glass_flag = !shoot.shoot_glass_flag;//开倍镜
  }
}

static void rc_shoot_cmd(uint8_t single_fir, uint8_t single_fir_after)
{

  if (single_fir)
  {
    shoot.c_shoot_time = HAL_GetTick();//发弹计时
    shoot.shoot_cmd   = 1;
		km.kb_enable = 0;
  }
	
	/* 遥控部分发弹防连发保险判断 */
	if (single_fir_after && (HAL_GetTick() - shoot.c_shoot_time >= 70)) //！！！70ms后取消发弹，强制垂直摩擦轮停转
  {
    shoot.shoot_cmd   = 0;
		km.kb_enable = 0;
  }

}
void remote_ctrl_shoot_hook(void)
{

  //水平摩擦轮控制
  rc_fric_ctrl(RC_CTRL_FRIC_WHEEL);
	
	//垂直摩擦轮控制
  rc_shoot_cmd(RC_SINGLE_SHOOT, RC_SINGLE_SHOOT_AFTER);
  
  //使能键盘鼠标
  kb_enable_hook();
}
//------------------------------------------------------------------------------------------------------------------------//
static void kb_enable_hook(void)
{
  if (rc.sw2 == RC_UP)//RC_MI
	  km.kb_enable = 1;
  else
    km.kb_enable = 0;
}

