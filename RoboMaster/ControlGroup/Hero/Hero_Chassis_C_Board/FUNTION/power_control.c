#include "power_control.h"
#include "chassis_task.h"
#include "pid.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "math.h"

void get_buffer(float *chassis_power_buffer);		   // 获取缓冲焦耳
void get_chassis_max_power(uint16_t *max_power_limit); // 获取限定功率

uint8_t cap_state = 0;
float Charge_factor = 1;// 1-Charge_factor = 电容充电百分比

float Chassis_Power_Control(chassis_t *chassis_power_control)
{
	uint16_t max_power_limit = 40; // 初始给定40w(没意义) -- 用于计算限定功率

	float chassis_max_power = 0; // 底盘最大给定功率
	float input_power = 0;		 // 限定功率（来自电管）
	float initial_give_power[4]; // 原先将要达到的功率
	float initial_total_power = 0;
	float scaled_give_power[4];
	float total_give = 0; // 最终给予的功率

	float chassis_power_buffer = 0.0f;

	float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55 --- n*toque_coefficient = 转矩/9.55 ， rpm*n*toque_coefficient = Pm(机械功率)

	float k2 = 1.23e-07;  // k2
	float k1 = 1.453e-07; // k1
	float constant = 4.081f;


	get_buffer(&chassis_power_buffer);							   // 获取缓冲焦耳
	pid_calc(&pid_chassis_power_buffer, chassis_power_buffer, BUFFER_ENERGY_REF); // 缓冲能量环，把缓冲焦耳压至BUFFER_ENERGY_REF，结果为负
	get_chassis_max_power(&max_power_limit);					   // 获取当前等级的功率限制
	input_power = max_power_limit - pid_chassis_power_buffer.out;  // 更新当前电管最大输出功率

	if(judge_recv_mesg.power_heat_data.buffer_energy<=10)
			send_cap_power_can(0);
	else
		send_cap_power_can(input_power * 100); // 将限制功率发送给功控板(单位:w)

	// 选择是否消耗电容
	if (FAST_SPD || rc.ch2 == 660)
	{
		cap_state = 1;
	}
	else
	{
		cap_state = 0;
	}

	if (chassis_power_control->CapData[1] > CAP_LOW) // 电容>16V
	{
		if (cap_state == 0) // 关闭电容加速
		{
			chassis_max_power = (input_power + 5)*Charge_factor; // 给定底盘的最大给定功率略大于限制功率，防止电容一直满电导致功率浪费
		}
		else // 开启电容加速
		{
			chassis_max_power = input_power + 200;
		}
	}
	else // 电容电量不足，底盘最大给定功率 = 限制功率
	{
		chassis_max_power = input_power*Charge_factor;
	}

	for (uint8_t i = 0; i < 4; i++) // 计算每个轮子原本将要达到的功率
	{
		// Pin = Pm + k1w^2 + k2tao^2 + constant
		// 拟合公式:Pin = Pm + k1*w^2 + k2*Icmd^2 + constant(用给定3508的n代替tao)
		initial_give_power[i] = chassis_power_control->current[i] * toque_coefficient * chassis_power_control->wheel_spd_fdb[i] +
								k1 * chassis_power_control->wheel_spd_fdb[i] * chassis_power_control->wheel_spd_fdb[i] +
								k2 * chassis_power_control->current[i] * chassis_power_control->current[i] + constant;

		if (initial_give_power[i] < 0) // 产生反生电动势，则不加入总功率的计算
			continue;
		initial_total_power += initial_give_power[i]; // 底盘原先将要达到的功率
	}
	total_give = initial_total_power;


	if (initial_total_power > chassis_max_power) // 底盘将要超功率--进行功率控制， 若没超，则继续pid运算至超功率
	{
		float power_scale = chassis_max_power / initial_total_power; // 缩放系数，用每个电机的当前功率乘以缩放系数，得到电机的期望功率，（功率总和 = chassis_max_power）
		total_give = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // 获取每个轮子缩放后的期望功率
			total_give += scaled_give_power[i];
			if (scaled_give_power[i] < 0) // 放出功率的就不参与计算
			{
				continue;
			}
			//根据当前转速，计算出能让电机达到期望功率的tao
			float a = k2;
			float b = toque_coefficient * chassis_power_control->wheel_spd_fdb[i];
			float c = k1 * chassis_power_control->wheel_spd_fdb[i] * chassis_power_control->wheel_spd_fdb[i] - scaled_give_power[i] + constant;

			if (chassis_power_control->current[i] > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					chassis_power_control->current[i] = 16000;
				}
				else
					chassis_power_control->current[i] = temp;
			}
			else
			{
				float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_power_control->current[i] = -16000;
				}
				else
					chassis_power_control->current[i] = temp;
			}
		}
	}

	return total_give;
}

void get_buffer(float *chassis_power_buffer)
{
	if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 45 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
		*chassis_power_buffer = judge_recv_mesg.power_heat_data.buffer_energy;
	else
		*chassis_power_buffer = BUFFER_ENERGY_REF; // 让焦耳环pid输出为0
}
void get_chassis_max_power(uint16_t *max_power_limit)
{
	if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 45 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
		*max_power_limit = judge_recv_mesg.game_robot_state.chassis_power_limit;
	else
		*max_power_limit = Debug_Power; // 没接入裁判系统，电管功率输出限定40w
}
