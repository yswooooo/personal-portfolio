#ifndef _pid_H
#define _pid_H

#include "stm32f4xx.h"

typedef enum
{
	INIT,
	DONE,
}INIT_STATUS;

enum
{
	NOW_ERR = 0,
	LAST_ERR,
	LLAST_ERR,
};
typedef enum 
{
	MOTTOR_NORMAL = 0 ,
	MOTTOR_BLOCKED = 1, 
}ERROR_TYPE_e;
typedef struct
{
	ERROR_TYPE_e ERROR_TYPE;
	uint16_t ERRORCount;
}ERROR_Handler_t;

typedef struct pid
{
	float set;
	float get;
	float error[3];
  
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iterm;
	float iout;
	float dout;
	float out; 
	
	
	int32_t maxout;
	int32_t integral_limit;
  float output_deadband;//ËÀÇø
	
	void (*f_pid_init)(struct pid *pid_t,
										float p,
										float i,
										float d,
										int32_t max_out,
										int32_t integral_limit);
	void (*f_pid_reset)(struct pid *pid_t,
										float p,
										float i,
										float d);
	
	ERROR_Handler_t 	ERROR_Handler; //PID¶Â×ª¼ì²â
}pid_t;


float pid_calc(pid_t *pid,float get,float set);
float fuzzy_pid_calc(pid_t *pid, float get, float set);
void PID_Clear(pid_t *pid);
void PID_Struct_Init(pid_t *pid,
										 float p,
										 float i,
										 float d,
										 int32_t max_out,
										 int32_t integral_limit,
										 INIT_STATUS init_status);

extern pid_t pid_yaw;
extern pid_t pid_pit;
										 
extern pid_t pid_yaw_spd;
extern pid_t pid_pit_spd;
										 
extern pid_t pid_trigger_angle;
extern pid_t pid_trigger_spd;
										 
extern pid_t pid_fric[4];
										 
extern pid_t pid_glass_spd ;
extern pid_t pid_glass;
										 
extern pid_t pid_imu_tmp;                   
#endif


