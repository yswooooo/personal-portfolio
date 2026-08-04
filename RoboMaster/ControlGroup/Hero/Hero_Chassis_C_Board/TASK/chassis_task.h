#ifndef _chassis_task_H
#define _chassis_task_H


#include "stm32f4xx.h"


typedef struct
{
  uint8_t         dodge_ctrl;
	uint8_t					separt_ctrl;
	uint8_t					chassis_flow;
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // 
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
  int16_t         current[4];
	double						CapData[4];
	uint8_t					cap_volt_warning;
  uint8_t fast_flag;		  // 加速状态标志位
  uint8_t Speed_up;			  // 加速键标志位
  float ob_total_power;  //当前底盘总功率
  float cap_store;       //// 电容存储大小(给定初值为24)
}chassis_t;

extern chassis_t chassis;

void chassis_task(void *parm);
void chassis_param_init(void);

static void chassis_normal_handler(void);
static void chassis_separate_handler(void);
static void chassis_dodge_handler(void);
static void chassis_stop_handler(void);
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

#endif
