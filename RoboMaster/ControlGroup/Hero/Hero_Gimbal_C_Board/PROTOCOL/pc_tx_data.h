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
 * @brief  1 + 4*3 = 13字节
 */
#pragma pack(1)
typedef struct
{
  uint8_t robot_color : 1;  // 敌方颜色
  uint8_t task_mode : 3;    // 当前任务模式
  uint8_t visual_valid : 1; // 视觉有效位
  uint8_t direction : 2;    // 扩展装甲板方向
  uint8_t bullet_level : 1; // 弹速
  float robot_pitch;        // 当前pitch位置
  float robot_yaw;          // 当前yaw位置 
  float time_stamp;         // 时间
} robot_tx_data;
#pragma pack()

extern fifo_s_t  pc_txdata_fifo;
extern robot_tx_data pc_send_mesg;

void pc_tx_param_init(void);
void pc_send_data_packet_pack(void);
void get_upload_data(void);
void get_infantry_info(void);


#endif
