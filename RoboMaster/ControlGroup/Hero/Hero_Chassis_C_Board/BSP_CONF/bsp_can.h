/************************************************************************************************************
底盘开发板——can1：
						接收：
						底盘电机  			 CAN_3508_M1_ID       = 0x201,
											CAN_3508_M2_ID       = 0x202,
											CAN_3508_M3_ID       = 0x203,
											CAN_3508_M4_ID       = 0x204,

						拨盘电机 			 CAN_TRIGGER_MOTOR_ID1 = 0x207,
											CAN_TRIGGER_MOTOR_ID2 = 0x208,
						电容      		CAN_SUPER_CAP_ID		 = 0x211,
						发送：
						底盘电机  CAN_CHASSIS_ALL_ID   = 0x200,
						拨盘电机 	 CAN_TRIG_ALL_ID			=		0x1ff,
						电容			CAN_CAP_POWER_ID	   = 0x210,
					——can2：
						接收：
						yaw轴电机 CAN_YAW_MOTOR_ID     = 0x20a,
						云台开发板 GIMBAL_MASTER_ID     = 0x101,
						发送：
						云台开发板 CAN_ANGLE_REF_ID 				= 0x306,
											 CAN_MODE_ID							= 0x307,
云台开发板——can1：
						接收：
						yaw轴电机 CAN_YAW_MOTOR_ID     = 0x20a,
						底盘开发板 CAN_ANGLE_REF_ID 				= 0x306,
											 CAN_MODE_ID							= 0x307,
						发送:
						yaw轴电机  CAN_GIMBAL_ALL_ID    = 0x2ff,
            底盘开发板 CAN_TO_CHASSIS_ID    = 0x101,					 
					——can2：
						接收：
						摩擦轮 		CAN_FRIC_M1_ID       = 0x205,
											CAN_FRIC_M2_ID       = 0x206,
						pit轴电机 CAN_PIT_MOTOR_ID     = 0x20b,
						上拨盘电机 CAN_TRIG_ID          = 0x208,
						发送：
						pit轴电机  CAN_GIMBAL_ALL_ID    = 0x2ff,
						摩擦轮、上拨盘  CAN_FRIC_ALL_ID      = 0x1ff,
**************************************************************************************************************/
#ifndef _bsp_can_H
#define _bsp_can_H

#include "stm32f4xx.h"

#define FILTER_BUF 5
/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

/* CAN send and receive ID */
typedef enum
{
	CAN_CHASSIS_ALL_ID   = 0x200,
	CAN_3508_M1_ID       = 0x201,//RF
	CAN_3508_M2_ID       = 0x202,//LF
	CAN_3508_M3_ID       = 0x203,//LB
	CAN_3508_M4_ID       = 0x204,//RB
	
  	CAN_TRIG_ALL_ID		  = 0x1ff,
	CAN_TRIGGER_MOTOR_ID1 = 0x207,//FRONT
	CAN_TRIGGER_MOTOR_ID2 = 0x208,//REAR

	CAN_CAP_POWER_ID	   = 0x210,
	CAN_SUPER_CAP_ID		 = 0x211,
} can1_msg_id_e;

typedef enum //!!!
{
	CAN_YAW_MOTOR_ID     = 0x20a,
	GIMBAL_MASTER_ID     = 0x101,
	
	CAN_ANGLE_REF_ID 	= 0x306,
	CAN_MODE_ID			= 0x307,
	CAN_SHOOT_CMD		=0x308,
} can2_msg_id_e;

typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_pit;
extern moto_measure_t moto_yaw;
extern moto_measure_t moto_loader[2];


void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message);
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message);

void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void send_trig_cur1(int16_t iq7,int16_t iq8);
void send_cap_power_can(uint16_t tempower);
void send_servant_angle(float yaw_ref, float pit_ref);
void send_servant_mode(char sw1, char sw2, char firc_flag, char action_flag);
void pit_encode(float *angle_flag);
void send_bullet_shoot_cmd(void);
#endif 

