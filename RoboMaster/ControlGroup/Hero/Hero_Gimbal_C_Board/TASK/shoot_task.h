#ifndef _shoot_task_H
#define _shoot_task_H


#include "stm32f4xx.h"

typedef enum
{
	SHOOTSTOP_MODE    = 0, //Ê§ÄÜ
	SHOOTMAD_MODE			= 1,

}shoot_para_e;

typedef __packed struct
{
  /* shoot task relevant param */

	uint8_t      last_para_mode; //Ä£Ê½
	uint8_t			 para_mode;

  uint8_t      fric_wheel_run; 		//run or not
  uint16_t     fric_wheel_spd_front_ref;
  uint16_t     fric_wheel_spd_rear_ref;
	
} shoot_t;

typedef __packed struct
{
  /* trigger motor param */
  int32_t   angle_ref;
  int32_t   spd_ref;

} trigger_t;

typedef __packed struct
{
  /* glass motor param */
  int32_t   angle_ref;
  int32_t   spd_ref;
  
} glass_t;

typedef enum
{
  SHOOT_CMD,
  FRIC_CTRL,
} shoot_type_e;

extern shoot_t   shoot;
extern trigger_t trig_ref;

void shoot_task(void *parm);

void shoot_param_init(void);
void get_last_shoot_mode(void);
static void shoot_para_ctrl(void);
static void fric_wheel_ctrl(void);
static void turn_on_friction_wheel(int16_t lspd,int16_t rspd);
static void turn_off_friction_wheel(void);


#endif

