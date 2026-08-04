#ifndef _keyboard_H
#define _keyboard_H


#include "stm32f4xx.h"

/* control key definition */
//      direction  key
#define FORWARD    (rc.kb.bit.W)
#define BACK       (rc.kb.bit.S)
#define LEFT       (rc.kb.bit.A)
#define RIGHT      (rc.kb.bit.D)
//      speed      key
#define FAST_SPD   (rc.kb.bit.SHIFT)
#define SLOW_SPD   (rc.kb.bit.CTRL)

/*小陀螺模式*/
#define KB_DODGE_CTRL (rc.kb.bit.E)
#define KB_DODGE_CTRL_CLOSE (rc.kb.bit.E && rc.kb.bit.CTRL)

/*鼠标右键长按自瞄*/
#define TRACK_CTRL (km.r_mouse_sta == MOUSE_LONG)

//      shoot relevant       key or mouse operation
#define KB_SINGLE_SHOOT     (km.l_mouse_sta == MOUSE_ONCE)
#define KB_CONTINUE_SHOOT   (km.l_mouse_sta == MOUSE_LONG)

/*摩擦轮开关*/
#define KB_OPEN_FRIC_WHEEL  (rc.kb.bit.Q)
#define KB_CLOSE_FIRC_WHEEL (rc.kb.bit.Q && rc.kb.bit.CTRL)

/*倍镜*/
#define KB_GLASS_CTRL		(rc.kb.bit.X)
#define KB_GLASS_CLOSE    (rc.kb.bit.X && rc.kb.bit.CTRL)

/*扩展装甲板*/
#define LEFT_OFFSET		(rc.kb.bit.C)
#define RIGHT_OFFSET		(rc.kb.bit.V)
/*分离模式*/
#define KB_SEPART_OPEN      (rc.kb.bit.R) 
#define KB_SEPART_CLOSE     (rc.kb.bit.R && rc.kb.bit.CTRL)

/*自爆不限热量模式(后备隐藏能源)*/
#define KB_SHOOT_BOOM			    (rc.kb.bit.B)
#define KB_SHOOT_BOOM_STOP	   (rc.kb.bit.B && rc.kb.bit.CTRL)
/*图传角度控制*/
#define KB_IMAGE			    (rc.kb.bit.F)
#define KB_IMAGE_STOP	        (rc.kb.bit.F && rc.kb.bit.CTRL)
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
//#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 		0x0020
//#define Q 			0x0040
//#define E				0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B				0x8000
/******************************************************/

typedef enum 
{
  MOUSE_RELEASE,
  MOUSE_PRESS,
  MOUSE_DONE,
  MOUSE_ONCE,
  MOUSE_LONG,
  
}MOUSE_STATUS;

typedef struct
{
  MOUSE_STATUS l_mouse_sta;
  MOUSE_STATUS r_mouse_sta;
  uint16_t l_cnt;
  uint16_t r_cnt;
  
  uint8_t kb_enable;//
  
  float vx_limit_speed;
  float vy_limit_speed;
  float vw_limit_speed;
  
  /*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v;
  
}kb_ctrl_t;

extern kb_ctrl_t km;

void keyboard_global_hook(void);
void keyboard_chassis_hook(void);
void keyboard_shoot_hook(void);
void keyboard_gimbal_hook(void);

#endif
