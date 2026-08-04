#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
// #include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "data_packet.h"
#include "arm_math.h"
#include "power_control.h"
#include "bsp_iwdg.h"  
//float ob_total_power;

UBaseType_t chassis_stack_surplus;



extern TaskHandle_t can_msg_send_Task_Handle;

extern pid_t pid_chassis_power_buffer;//缓冲焦耳
//extern uint8_t twist_volt;//小陀螺标志位
extern pid_t pid_chassis_vw ;//小陀螺pid

chassis_t chassis = {	
						.cap_store = 24.0f, 
						.fast_flag = 0, 
						.Speed_up = 0 
					};//底盘控制结构体


float power_new_pid[3] = {3.0f, 1.0f, 0.0f}; //新功率算法PID
float chassis_power_buffer_pid[3] = {1, 0, 0}; // 缓冲焦耳控制PID
float chassis_vw_pid[3] = {0.0f, 0.5f, 0.0f};	   // 小陀螺转速控制PID
float chassis_yaw_cali_pid[3] = {7.0f, 0.0f, 10.0f};//底盘跟随YAWPID
float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},//四轮速度环PID
															 {10.5, 0.0f, 0},
															 {10.5, 0.0f, 0},
															 {10.5, 0.0f, 0}};

//float cap_store 24;		  // 电容存储大小(给定初值为24)

//uint8_t fast_flag = 0;		  // 加速状态标志位
//uint8_t Speed_up = 0;			  // 加速键标志位



void chassis_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)INFO_GET_CHASSIS_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{

			if (Signal & INFO_GET_CHASSIS_SIGNAL)
			{
				/*底盘vw旋转的pid*/
				PID_Struct_Init(&pid_chassis_cali_angle, chassis_yaw_cali_pid[0], chassis_yaw_cali_pid[1], chassis_yaw_cali_pid[2], MAX_CHASSIS_VR_SPEED, 10, DONE);
				PID_Struct_Init(&pid_chassis_power_buffer, chassis_power_buffer_pid[0], chassis_power_buffer_pid[1], chassis_power_buffer_pid[2], 50, 10, DONE);
				PID_Struct_Init(&pid_chassis_vw, chassis_vw_pid[0], chassis_vw_pid[1], chassis_vw_pid[2], 570, 570, DONE);

				/*底盘vx,vy平移的pid*/
				for (int i = 0; i < 4; i++)
					PID_Struct_Init(&pid_chassis_vx_vy_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 16000, 1500, DONE);

				if (chassis_mode != CHASSIS_RELEASE && gimbal.state != GIMBAL_INIT_NEVER) // 云台归中之后底盘才能动
				{
					switch (chassis_mode)
					{
					/*云台底盘跟随模式*/
					case CHASSIS_NORMAL_MODE:
					{
						chassis_normal_handler();
					}
					break;
					/*小陀螺模式*/
					case CHASSIS_DODGE_MODE:
					{
						chassis_dodge_handler();
					}
					break;
					/*底盘停止模式*/
					case CHASSIS_STOP_MODE:
					{
						chassis_stop_handler();
					}
					break;

					default:
					{
					}
					break;
					}



					//麦克纳姆轮速度映射结算至底盘四个轮子
					mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);

					for (int i = 0; i < 4; i++)
						chassis.current[i] = pid_calc(&pid_chassis_vx_vy_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);

					chassis.ob_total_power = Chassis_Power_Control(&chassis); // 功率控制


					if (!chassis_is_controllable())
						memset(chassis.current, 0, sizeof(chassis.current));

					memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
				}
				else
					memset(glb_cur.chassis_cur, 0, sizeof(glb_cur.chassis_cur));
				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)CHASSIS_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
			}
		}

		chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

//extern uint8_t Speed_up;

//新功率算法
/**
 * @brief 功率控制
 * @param power_pid  &pid_power
 * @param power_vx  &dodge_chassis_vx
 * @param power_vy &dodge_chassis_vy
 * @param real_time_Cap_remain chassis.CapData[1]
 * @param real_time_Cap_can_store cap_store
 * @param judge_power_limit (float)judge_recv_mesg.game_robot_state.chassis_power_limit
 */
void chassis_power_contorl(pid_t *power_pid,float *power_vx,float *power_vy,float real_time_Cap_remain,float real_time_Cap_can_store,float judge_power_limit)
{
    static  float max = 3300;
    static float min = 1150;//1000;
    static float Cap_low = CAP_LOW;      // 何时停止加速
	float Ref_temp = 0;//功率缓启动临时变量
    float cap_flag = real_time_Cap_can_store - 18.0f; // 18~real_time_Cap_can_store这一段进行按比例赋值



		if (real_time_Cap_remain < Cap_low)
				chassis.Speed_up = 1; // 一旦检测到电容低则自动关闭加速
		
		if(!SLOW_SPD)
		{
			Charge_factor = 1.0f;
		}
		else//降速给底盘充电
		{
			Charge_factor = 0.6f;// 1-0.6=40% 的电量给电容充电
		}	
		
		if(km.vy != 0 || rm.vy != 0 || km.vx != 0 || rm.vx != 0)//底盘输入不为0,才进行pid运算（防止底盘静止时，pid->out积满）
		{
				if (chassis.Speed_up == 1)//不加速				
				{
					if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
					{
						if((judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor - chassis.ob_total_power > Deta_Power)//功率相差过大
						{
							Ref_temp = chassis.ob_total_power + Deta_Power;							
							pid_calc(power_pid,chassis.ob_total_power,Ref_temp);
						}
						else
							pid_calc(power_pid, chassis.ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
					}
					else//没接裁判系统
						pid_calc(power_pid,chassis.ob_total_power,(Debug_Power+5)*Charge_factor);//没有连接裁判系统，小车底盘期望功率达到45w			
				}
			
		}
		if(chassis.Speed_up == 1)
		{
			if (km.vy > 0 || rm.vy > 0)
			{
				 *power_vy = (min + power_pid->out);
				 VAL_LIMIT(*power_vy, min * 0.7f, max);
			}
			else if (km.vy < 0 || rm.vy < 0)
			{
				 *power_vy = -(min + power_pid->out);
				 VAL_LIMIT(*power_vy, -max, -min * 0.7f);
			}
			else
				*power_vy = 0;

			if (km.vx > 0 || rm.vx > 0)
			{
					*power_vx = (min + power_pid->out);
				 VAL_LIMIT(*power_vx, min, max);
			}
			else if (km.vx < 0 || rm.vx < 0)
			{
				*power_vx = -(min + power_pid->out);
				VAL_LIMIT(*power_vx, -max, -min);
			}
			else
				*power_vx = 0;
		}
		else if(chassis.Speed_up == 0)
		{
			if (km.vy > 0 || rm.vy > 0)
			{
				 *power_vy = CHASSIS_KB_MAX_SPEED_Y;
			}
			else if (km.vy < 0 || rm.vy < 0)
			{
				 *power_vy = -CHASSIS_KB_MAX_SPEED_Y;
			}
			else
				*power_vy = 0;

			if (km.vx > 0 || rm.vx > 0)
			{
					*power_vx = CHASSIS_KB_MAX_SPEED_X;
			}
			else if (km.vx < 0 || rm.vx < 0)
			{
				*power_vx = -CHASSIS_KB_MAX_SPEED_X;
			}
			else
				*power_vx = 0;
		}
		
	if (chassis.fast_flag && (rc.ch2 == 660 || FAST_SPD)) // 快速
	{
		chassis.Speed_up = 0;
	}
	else // 正常速度
	{
		chassis.Speed_up = 1;
	}

	if (real_time_Cap_remain > Cap_low && !FAST_SPD)
	{
		chassis.fast_flag = 1;
	} // 防止加速过程中反复开关加速模式（抽搐）
	if (real_time_Cap_remain < Cap_low)
	{
		chassis.fast_flag = 0;
	}
}

// 普通模式
static void chassis_normal_handler(void)
{
	float normal_chassis_vx, normal_chassis_vy;//临时vx、vy速度变量
	// 计算底盘x、y轴上的速度
	chassis_power_contorl(&pid_power, &normal_chassis_vx, &normal_chassis_vy,chassis.CapData[1], chassis.cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	if (chassis_mode == CHASSIS_NORMAL_MODE)
	{
		chassis.vw = (-pid_calc(&pid_chassis_cali_angle, gimbal.sensor.yaw_relative_angle, 0.0f));//底盘跟随云台
	}
	//结算出最终速度
	chassis.vx = normal_chassis_vx;
	chassis.vy = normal_chassis_vy;
}

// 小陀螺模式
#define dodge_min  150
#define dodge_max  570
static void chassis_dodge_handler(void)
{
	float dodge_angle;
//	float dodge_min = 150;
//	float dodge_max = 570;
	float dodge_chassis_vx, dodge_chassis_vy;

	if (last_chassis_mode != CHASSIS_DODGE_MODE)
	{
		PID_Clear(&pid_chassis_vw);
	}

	/*小陀螺*/
	dodge_angle = gimbal.sensor.yaw_relative_angle;
	//
	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // 小陀螺时遥控方向降低速度
	{
		dodge_chassis_vx = rm.vx * 0.5f;
		dodge_chassis_vy = rm.vy * 0.5f;
	}
	else
		// 键盘时则通过功率算法来控制速度，防止掉电容
		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy,chassis.CapData[1], chassis.cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

	if (chassis.Speed_up == 1)//不加速				
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,chassis.ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
		}
		else//没接裁判系统
			pid_calc(&pid_chassis_vw,chassis.ob_total_power,(Debug_Power+5)*Charge_factor);//没有连接裁判系统，小车底盘期望功率达到45w
	}
	else if (chassis.Speed_up == 0)//按下加速功率多100w
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,chassis.ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit+105));
		}
		else//没接裁判系统
		{
			pid_calc(&pid_chassis_vw,chassis.ob_total_power,105+Debug_Power);//没有连接裁判系统，小车底盘期望功率达到150w
		}
	}	

	chassis.vw = dodge_min+pid_chassis_vw.out;
	if (chassis.vw < dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw > dodge_max)
		chassis.vw = dodge_max;
	
	if(  ((rc.sw1 == RC_MI) && (rc.sw2 == RC_UP) && (rc.iw <= IW_UP)))
	{
		chassis.vw =-(dodge_min+pid_chassis_vw.out);
	if (chassis.vw < (-dodge_min))
		chassis.vw = (-dodge_min);
	if (chassis.vw > (-dodge_max))
		chassis.vw = (-dodge_max);
	}
}

// 停止模式
static void chassis_stop_handler(void)
{
	// for (int i = 0; i < 4; i++)
	// {
	// 	chassis.current[i] = pid_calc(&pid_chassis_vx_vy_spd[i], chassis.wheel_spd_fdb[i], 0);
	// }

	chassis.vy = 0;
	chassis.vx = 0;
	chassis.vw = 0;
}
// 初始化
void chassis_param_init(void)
{
	memset(&chassis, 0, sizeof(chassis));

	/*底盘vw旋转的pid*/
	PID_Struct_Init(&pid_chassis_cali_angle, chassis_yaw_cali_pid[0], chassis_yaw_cali_pid[1], chassis_yaw_cali_pid[2], MAX_CHASSIS_VR_SPEED, 50, INIT);
  PID_Struct_Init(&pid_power, power_new_pid[0], power_new_pid[1], power_new_pid[2], 3000, 2000, INIT);//新功率算法

	/*底盘vx,vy平移的pid*/
	for (int i = 0; i < 4; i++)
		PID_Struct_Init(&pid_chassis_vx_vy_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 10000, 500, INIT);

}

/**
 * @brief mecanum chassis velocity decomposition
 * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
 *        output: every wheel speed(rpm)
 * @trans 输入：		前后左右的量
 *				 输出：		每个轮子对应的速度
 * @note  1=FR 2=FL 3=BL 4=BR
 * @work	 分析演算公式计算的效率
 */

#define rotate_ratio_fr ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF
#define rotate_ratio_fl ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF
#define rotate_ratio_bl ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF
#define rotate_ratio_br ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF
#define wheel_rpm_ratio 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO)

static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]) // 底盘解算，得到底盘获得相应速度需要的四个电机值
{
	//static float rotate_ratio_fr;
	//static float rotate_ratio_fl;
	//static float rotate_ratio_bl;
	// static float rotate_ratio_br;
	// static float wheel_rpm_ratio;

	//taskENTER_CRITICAL();
	//@work
	//rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	//rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	//rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	// rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	// wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO);
	//taskEXIT_CRITICAL();

	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
	VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
	/*小陀螺以外的模式，vw限制正常*/
	if (chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED); // deg/s

	int16_t wheel_rpm[4];
	float max = 0;

	wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
	wheel_rpm[1] = (vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
	wheel_rpm[2] = (vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
	wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

	// find max item
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(wheel_rpm[i]) > max)
			max = abs(wheel_rpm[i]);
	}
	// equal proportion
	if (max > MAX_WHEEL_RPM)
	{
		float rate = MAX_WHEEL_RPM / max;
		for (uint8_t i = 0; i < 4; i++)

			wheel_rpm[i] *= rate;
	}
	memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}


