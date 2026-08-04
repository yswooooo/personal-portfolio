#ifndef _chassis_task_H
#define _chassis_task_H


#include "stm32f4xx.h"

//#define TWIST_ANGLE    45
#define CHASSIS_PERIOD 10
#define TWIST_PERIOD   1200//1300

typedef struct
{
  uint8_t         dodge_ctrl;
	uint8_t					chassis_flow;
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // 
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
  int16_t         current[4];
  int16_t         position_ref;
	float						CapData[4];
}chassis_t;

extern chassis_t chassis;

void chassis_task(void *parm);
void chassis_param_init(void);


#endif
