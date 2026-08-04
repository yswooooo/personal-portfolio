#ifndef _detect_task_H
#define _detect_task_H


#include "stm32f4xx.h"

typedef enum
{
  BOTTOM_DEVICE        = 0,
  CHASSIS_M1_OFFLINE   = 1,
  CHASSIS_M2_OFFLINE   = 2,
  CHASSIS_M3_OFFLINE   = 3,
  CHASSIS_M4_OFFLINE   = 4,
  GIMBAL_YAW_OFFLINE   = 5,
  GIMBAL_PIT_OFFLINE   = 6,
  TRIGGER_MOTO_OFFLINE = 7,
  
  FRI_MOTO1_OFFLINE    = 8,
  FRI_MOTO2_OFFLINE    = 9,
  FRI_MOTO3_OFFLINE    = 10,
  FRI_MOTO4_OFFLINE    = 11,

	IMU_OFFLINE					 = 12,
  REMOTE_CTRL_OFFLINE  = 13,
	GLASS_MOTO_OFFLINE	 = 14,
  PC_SYS_OFFLINE       = 15,
  ERROR_LIST_LENGTH    = 16,
}err_id;

typedef struct
{
  uint32_t last_times;
  uint32_t delta_times;
  uint16_t set_timeout;
  
}detect_param_t;

typedef struct
{
  uint8_t err_exist;
  detect_param_t param; 
  
}err_status;

typedef struct
{
  uint8_t err_now_id[ERROR_LIST_LENGTH];
  err_status list[ERROR_LIST_LENGTH];
  
}global_err_t;

extern global_err_t global_err;

void detect_task(void *parm);
void detect_param_init(void);
void err_detector_hook(int err_id);
void module_offline_detect(void);
void module_offline_callback(void);

uint8_t gimbal_is_controllable(void);
uint8_t chassis_is_controllable(void);
uint8_t shoot_is_controllable(void);

#endif

