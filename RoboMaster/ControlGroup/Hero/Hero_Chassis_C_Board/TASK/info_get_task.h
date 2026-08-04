#ifndef info_get_task_H
#define info_get_task_H



#include "stm32f4xx.h"


void info_get_task(void *parm);

static void get_chassis_info(void);
static void get_structure_param(void);
static void get_gimbal_info(void);
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);
static void get_shoot_info(void);
static void get_global_last_info(void);

#endif
