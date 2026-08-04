#ifndef _shoot_task_H
#define _shoot_task_H


#include "stm32f4xx.h"

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,
} shoot_mode_e;

typedef enum
{
	SHOOTSTOP_MODE    = 0,
	SHOOTMAD_MODE			= 1,

}shoot_para_e;


typedef __packed struct
{
  /* shoot task relevant param */
  shoot_mode_e ctrl_mode;
	shoot_mode_e last_ctrl_mode;
  uint8_t      shoot_cmd;
  uint32_t     c_shoot_time;   		//count
	
	uint8_t      last_para_mode;
	uint8_t			 para_mode;
  uint8_t			 ball_storage_open;	//ball storage
	
  uint8_t      fric_wheel_run; 		//run or not
  uint16_t     fric_wheel_spd;
	uint8_t			 fric_wheel_compe;	//摩擦轮转速补偿
	uint16_t		 fric_launch_time;
	
	uint8_t 			shoot_boom_flag; //boom
	uint8_t       shoot_glass_flag; //glass
	uint8_t 		shoot_image_flag;//图传
} shoot_t;

typedef __packed struct
{
  /* trigger motor param */
  int32_t   angle_ref;
  int32_t   spd_ref;    //英雄无连发，故只用这一个参数做速度单闭环
  
} loader_t;

typedef enum
{
  SHOOT_CMD,
  FRIC_CTRL,
} shoot_type_e;

extern shoot_t   shoot;
extern loader_t loader[2];

void shoot_task(void *parm);

void shoot_param_init(void);
void get_last_shoot_mode(void);
static void shoot_bullet_handler(void);
static void shoot_para_ctrl(void);
static void fric_wheel_ctrl(void);
static void turn_on_friction_wheel(int16_t lspd,int16_t rspd);
static void turn_off_friction_wheel(void);



#endif

