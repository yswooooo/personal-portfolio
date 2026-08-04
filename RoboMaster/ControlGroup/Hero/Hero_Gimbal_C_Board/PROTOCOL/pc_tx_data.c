#include "pc_tx_data.h"
#include "pc_rx_data.h"
#include "judge_rx_data.h"//
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "keyboard.h"
#include "rc.h"
#include "shoot_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*小电脑发送*/
fifo_s_t  pc_txdata_fifo;
static SemaphoreHandle_t pc_txdata_mutex;
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

extern uint8_t left_offset_cmd;
extern uint8_t	right_offset_cmd;

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
      //pc_send_mesg.robot_pitch = gimbal.sensor.pit_relative_angle;//发送pitch电机与校准零点的相对角
	  pc_send_mesg.robot_pitch = gimbal.sensor.pit_gyro_angle;
      pc_send_mesg.robot_yaw = gimbal.sensor.yaw_gyro_angle;//发送陀螺仪的数据
			
      /*反小陀螺-打哨兵模式按键 长按C发送1，长按V发送2*/
      /* 松手发送0               向左补偿    向右补偿 */
      //direction:2 拓展装甲板标志位
//      if(left_offset_cmd == 1)  
//        pc_send_mesg.direction=  1;
//      else if(right_offset_cmd == 1)
//        pc_send_mesg.direction = 2;
//      else
//        pc_send_mesg.direction = 0;
		}break;

		default:
    {
			pc_send_mesg.task_mode = NORMAL_CTRL_MODE;
      pc_send_mesg.robot_pitch = gimbal.sensor.pit_gyro_angle;
      pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_gyro_angle; //默认发送陀螺仪角度
		}break;
	}
	
}
