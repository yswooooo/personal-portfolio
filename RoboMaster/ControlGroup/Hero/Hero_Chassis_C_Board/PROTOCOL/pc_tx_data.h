#ifndef _pc_tx_data_H
#define _pc_tx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_TX_FIFO_BUFLEN 500

typedef enum
{
	TRACK_AMOR_MODE			=0,
	BIG_BUFF_MODE				=1,
	NORMAL_CTRL_MODE		=2,
	SMALL_BUFF_MODE			=3,
}mode_t;

typedef enum
{
	red  = 0,
	blue = 1,
}enemy_color_t;

/**
 * @brief  电控->PC的数据段robot_tx_data (13字节) ，数据帧长：5 + 13 + 2 = 20 (字节)
 */
#pragma pack(1)
typedef struct
{
  uint8_t robot_color : 1;  // 颜色 (0/1)
  uint8_t task_mode : 3;    // 识别模式  （0 自瞄 / 1 大小符）
  uint8_t visual_valid : 1; // 视觉有效位 (0/1)
  uint8_t direction : 2;    // 拓展装甲板标志位 (0-3)
  uint8_t bullet_level : 1; // 弹速等级 1 2 3级
  float robot_pitch;        // 欧拉角(度)
  float robot_yaw;          // 欧拉角(度)
  float time_stamp;         // 电控时间戳(ms)
} robot_tx_data;
#pragma pack()

extern fifo_s_t  pc_txdata_fifo;
extern robot_tx_data pc_send_mesg;

void pc_tx_param_init(void);
void pc_send_data_packet_pack(void);
void get_upload_data(void);
void get_infantry_info(void);


#endif
