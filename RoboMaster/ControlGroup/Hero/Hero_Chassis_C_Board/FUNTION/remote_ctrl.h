#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"

/*底盘控制*/
#define RC_DODGE_MODE             ((rc.sw1 == RC_MI) && (rc.sw2 == RC_UP) && (rc.iw >= IW_DN)) ||  ((rc.sw1 == RC_MI) && (rc.sw2 == RC_UP) && (rc.iw <= IW_UP))

/*射击控制*/
#define RC_SINGLE_SHOOT    				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_SINGLE_SHOOT_AFTER  		 (rc.sw1 == RC_DN)
#define RC_CTRL_FRIC_WHEEL 				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
#define RC_CTRL_BALL_STOR	 				((rc.sw2 == RC_UP) && (rc.iw <= IW_UP))
#define RC_CTRL_BALL_STOR_CLOSE	 	((rc.sw2 == RC_UP) && (rc.iw >= IW_DN))


enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
	IW_UP = 694,		//	min 364		middle 1024	拨轮
	IW_DN = 1354,		//	max 1684
	IW_MI = 1024,
};

typedef struct
{
	/*记录拨杆上一次状态*/
  uint8_t last_sw1;
  uint8_t last_sw2;
} sw_record_t;

typedef struct
{
	/*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v;
} rc_ctrl_t;

extern rc_ctrl_t rm;
extern sw_record_t glb_sw;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_chassis_hook(void);
void remote_ctrl_gimbal_hook(void);
void remote_ctrl_shoot_hook(void);

#endif


