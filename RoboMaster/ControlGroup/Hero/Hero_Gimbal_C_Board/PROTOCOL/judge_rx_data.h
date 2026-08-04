#ifndef _judge_rx_data_H
#define _judge_rx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 500


typedef enum
{
	GAME_STATE_ID                = 0x0001,  //比赛状态数据
	GAME_RESULT_ID 	             = 0x0002,  //比赛结果数据
	ROBOT_SURVIVORS_ID           = 0x0003,	//比赛机器人存活数据
	EVENT_DATA_ID 				       = 0x0101,	//场地事件数据
	SUPPLY_PROJECTILE_ACTION_ID  = 0x0102,	//场地补给站动作标识数据
	SUPPLY_PROJECTILE_BOOK_ID    = 0x0103,	//场地补给站预约子弹数据
	GAME_ROBOT_STATE_ID          = 0x0201,	//机器人状态数据
	POWER_HEAT_DATA_ID           = 0x0202,	//实时功率热量数据
	ROBOT_POS_ID                 = 0x0203,	//机器人位置数据
	BUFF_MUSK_ID                 = 0x0204,	//机器人增益数据
	AERIAL_ROBOT_ENERGY_ID       = 0x0205,	//空中机器人能量状态数据
	ROBOT_HURT_ID                = 0x0206,	//伤害状态数据
	SHOOT_DATA_ID                = 0x0207,	//实时射击数据
	STUDENT_INTERACTIVE_ID       = 0x0301,	//机器人间交互数据 和 客户端通信
} judge_data_id_e;

/*裁判系统数据结构体*/

typedef __packed struct
{
 uint8_t game_type:4;
 uint8_t game_progress:4;
 uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;

typedef __packed struct
{
 uint16_t robot_legion;
} ext_game_robot_survivors_t;

typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
	
 uint8_t supply_projectile_num; //1.1版本
} ext_supply_projectile_action_t;

typedef __packed struct
{
 uint8_t supply_projectile_id;
 uint8_t supply_robot_id;    //1.1版本
 uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_heat0; 
 uint16_t shooter_heat1; 
} ext_power_heat_data_t;


typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_musk_t;

typedef __packed struct
{
 uint8_t energy_point;
 uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
	
 uint8_t interactive_data[50];
}ext_RX_student_interactive_header_data_t;

typedef struct
{
  ext_game_state_t                      	    game_state;               	 	   //0x0001
  ext_game_result_t                      	    game_result;               	     //0x0002
  ext_game_robot_survivors_t       			 	    game_robot_survivors;     		   //0x0003
  ext_event_data_t  										      event_data;                	     //0x0101
  ext_supply_projectile_action_t     	        supply_projectile_action;  	     //0x0102
  ext_supply_projectile_booking_t     	      supply_projectile_booking; 	     //0x0103
  ext_game_robot_state_t        		          game_robot_state;      		       //0x0201
  ext_power_heat_data_t   				            power_heat_data;     		 	       //0x0202
  ext_game_robot_pos_t   				              game_robot_pos;				           //0x0203
  ext_buff_musk_t						                  buff_musk;						           //0x0204
  aerial_robot_energy_t					              aerial_robot_energy;			       //0x0205
  ext_robot_hurt_t						                robot_hurt;						           //0x0206
  ext_shoot_data_t						                shoot_data;			           	     //0x0207
  ext_RX_student_interactive_header_data_t	  student_interactive_header_data; //0x0301

} judge_rxdata_t;

extern judge_rxdata_t judge_recv_mesg;
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);

#endif

