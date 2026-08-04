#ifndef _gimbal_task_H
#define _gimbal_task_H


#include "stm32f4xx.h"
#include "ramp.h"
typedef enum
{
  GIMBAL_INIT_NEVER,
  GIMBAL_INIT_DONE,
  NO_ACTION,
  IS_ACTION, 
}gimbal_state_t;


typedef struct
{
  /* 角度环的给定和反馈 */
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* 速度环的给定和反馈 */
  float yaw_spd_ref;
  float pit_spd_ref;
  float yaw_spd_fdb;
  float pit_spd_fdb;
} gim_pid_t;

typedef struct
{
  /* 相对角度，陀螺仪欧拉角 */
  float pit_relative_angle;
  float yaw_relative_angle;
  float pit_gyro_angle;
  float yaw_gyro_angle;
  /* 角速度 */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;


typedef struct
{
  uint8_t track_ctrl;
  
  gim_pid_t pid;
  gim_sensor_t sensor;
  gimbal_state_t state;
  gimbal_state_t last_state;
  
  /*从FLASH中读出的校准编码位*/
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  float         yaw_offset_angle;
  float 	 			yaw_angle;
  
}gimbal_t;

/*卡尔曼用，速度相关结构体*/
typedef struct  // speed_calc_data_t
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;


/*前馈控制参数*/
typedef struct
{
	//前馈补偿参数
	float a;
	float b;
	//缓存参数
	float rin;
	float lastRin;
	float perrRin;
}FFC;

/*简单速度前馈*/
typedef struct
{
  float Kc;
  float Outner_Expect[2];
  float out;
}Easy_FFC_t;

enum
{
  LAST = 0,
  NOW = 1,
};
extern gimbal_t gimbal;
extern uint8_t input_flag;
extern ramp_t pit_ramp;
extern ramp_t yaw_ramp;

void gimbal_task(void *parm);
void gimbal_param_init(void);

static void init_mode_handler(void);
static void nomarl_handler(void);
static void separate_handler(void);
static void dodge_handler(void);
static void track_aimor_handler(void);
//前馈控制器
void initFeedforwardParam(FFC *vFFC,float a,float b);
float getFeedforwardControl(FFC* vFFC,float v);
void Easy_FFC_Init(Easy_FFC_t* easy_ffc ,float K);
void Easy_FFC_Calc(Easy_FFC_t* easy_ffc ,float K ,float gimbal_expect_ref);
#endif

