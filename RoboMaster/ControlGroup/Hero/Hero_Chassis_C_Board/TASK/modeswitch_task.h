#ifndef _modeswitch_task_H
#define _modeswitch_task_H


#include "stm32f4xx.h"
#include "string.h"

#define MEMSET(flag,type) (memset((type*)flag,0,sizeof(type)))

/*主要控制模式*/
typedef enum
{
  RELEASE_CTRL,
  MANUAL_CTRL,
  SEMI_AUTOMATIC_CTRL,
}global_status;
/*云台控制模式*/
typedef enum
{
  GIMBAL_RELEASE,
  GIMBAL_INIT,
  GIMBAL_NORMAL_MODE,
  GIMBAL_SEPARATE_MODE,
  GIMBAL_DODGE_MODE,
  GIMBAL_SHOOT_BUFF,
  GIMBAL_TRACK_ARMOR,
}gimbal_status;
/*底盘控制模式*/
typedef enum
{
  CHASSIS_RELEASE,
  CHASSIS_NORMAL_MODE,
  CHASSIS_SEPARATE_MODE,
  CHASSIS_DODGE_MODE,
  CHASSIS_STOP_MODE,
  
}chassis_status;

typedef enum
{
  SHOOT_DISABLE,
  SHOOT_ENABLE,
  
}shoot_status;

enum
{
	GIMBAL_FOLLOW_NORMAL = 0,
	GIMBAL_FOLLOW_TRACE  = 1,
	GIMBAL_FOLLOW_DODGE  = 2,
	GIMBAL_FOLLOW_SEPERATE = 3,
};
enum
{
  OFF = 0,
  ON = 1,
};
enum
{
	LEFT_SWITCH = 0,
	RIGHT_SWITCH= 1,
};

enum
{
	GIMBAL_YAW_ANGLE = 0,
	GIMBAL_PITCH_ANGLE = 1,
};


extern global_status global_mode; 
extern global_status last_global_mode;

extern gimbal_status gimbal_mode;
extern gimbal_status last_gimbal_mode;

extern chassis_status chassis_mode;
extern chassis_status last_chassis_mode;

extern shoot_status shoot_mode;
extern shoot_status last_shoot_mode;

void mode_switch_task(void *parm);
void get_last_mode(void);
void get_main_mode(void);
void get_gimbal_mode(void);
void get_chassis_mode(void);
void get_shoot_mode(void);

#endif

