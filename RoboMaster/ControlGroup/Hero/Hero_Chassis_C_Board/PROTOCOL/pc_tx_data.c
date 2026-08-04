#include "pc_tx_data.h"
#include "pc_rx_data.h"
#include "judge_rx_data.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "keyboard.h"
#include "rc.h"
#include "shoot_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*小电脑发送*/
static SemaphoreHandle_t pc_txdata_mutex;
fifo_s_t  pc_txdata_fifo;
static uint8_t   pc_txdata_buf[PC_TX_FIFO_BUFLEN];

void pc_tx_param_init(void)
{
    /* create the judge_rxdata_mutex mutex  */  
  pc_txdata_mutex = xSemaphoreCreateMutex();
  
  /* judge data fifo init */
  fifo_s_init(&pc_txdata_fifo, pc_txdata_buf, PC_TX_FIFO_BUFLEN, pc_txdata_mutex);
}


/* data send */
robot_tx_data pc_send_mesg;

uint8_t pc_tx_packet_id = GIMBAL_DATA_ID;
int count_cali_count = 0;
void pc_send_data_packet_pack(void)
{
	get_upload_data();
	
	//SENTRY_DATA_ID 哨兵  UP_REG_ID小电脑帧头
	data_packet_pack(SENTRY_DATA_ID, (uint8_t *)&pc_send_mesg, 
							 sizeof(robot_tx_data), UP_REG_ID);

}

void get_upload_data(void)
{
  taskENTER_CRITICAL();
  
  get_infantry_info();
  
  taskEXIT_CRITICAL();
}

void get_infantry_info(void)
{
	uint8_t mode = gimbal_mode;
	/* 获取敌方颜色 */
	if(judge_recv_mesg.game_robot_state.robot_id>10)
		pc_send_mesg.robot_color = red;//敌方红		0
	else
		pc_send_mesg.robot_color = blue;//敌方蓝	1
  /* get gimable ctrl mode */
	switch(mode)
	{
		case GIMBAL_TRACK_ARMOR:
    { 
			pc_send_mesg.task_mode = TRACK_AMOR_MODE;		//自瞄模式
      pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;//0
      pc_send_mesg.robot_yaw = gimbal.sensor.yaw_gyro_angle;//发送陀螺仪的数据//0;(不发)
      /*反小陀螺-打哨兵模式按键 长按C发送1，长按V发送2*/
      /* 松手发送0               向左补偿    向右补偿 */
      //direction:2 拓展装甲板标志位

		}break;

		default:
    {
			pc_send_mesg.task_mode = NORMAL_CTRL_MODE;
      pc_send_mesg.robot_pitch = gimbal.sensor.pit_gyro_angle;
      pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_gyro_angle; //默认发送陀螺仪角度
		}break;
	}
	
}
