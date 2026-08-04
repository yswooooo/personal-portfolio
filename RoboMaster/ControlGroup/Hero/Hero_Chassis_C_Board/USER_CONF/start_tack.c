/*********************************************************
*                       _oo0oo_                          *
*                      o8888888o                         *
*                      88" . "88                         *
*                      (| -_- |)                         *
*                      0\  =  /0                         *
*                    ___/`---'\___                       *
*                  .' \\|     |// '.                     *
*                 / \\|||  :  |||// \                    *
*                / _||||| -:- |||||- \                   *
*               |   | \\\  -  /// |   |                  *
*               | \_|  ''\---/''  |_/ |                  *
*               \  .-\__  '-'  ___/-. /                  *
*            ___'. .'  /--.--\  `. .'___                 *
*         ."" '<  `.___\_<|>_/___.' >' "".               *
*         | | :  `- \`.;`\ _ /`;.`/ - ` : | |            *
*         \  \ `_.   \_ __\ /__ _/   .-` /  /            *
*    =====`-.____`.___ \_____/___.-`___.-'=====          *
*                       `=---='                          *
*                                                        *
*                                                        *
*     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~        *
*                                                        *
*               佛祖保佑         永无BUG                 *
**********************************************************/

#include "start_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "imu_task.h"
#include "comm_task.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "judge_task.h"
#include "pc_task.h"

#define START_TASK_SIZE 128
#define START_TASK_PRIO 2

#define GIMBAL_TASK_SIZE 128
#define GIMBAL_TASK_PRIO 5

#define CHASSIS_TASK_SIZE 256
#define CHASSIS_TASK_PRIO 5

#define SHOOT_TASK_SIZE 128
#define SHOOT_TASK_PRIO 5

#define CAN_MSG_SEND_TASK_SIZE 128
#define CAN_MSG_SEND_TASK_PRIO 6

#define MODE_SWITCH_TASK_SIZE 128
#define MODE_SWITCH_TASK_PRIO 4

#define INFO_GET_TASK_SIZE 128
#define INFO_GET_TASK_PRIO 4

#define DETECT_TASK_SIZE 128
#define DETECT_TASK_PRIO 4

#define IMU_TASK_SIZE 128
#define IMU_TASK_PRIO 4

#define JUDEG_TX_TASK_SIZE 256
#define JUDEG_TX_TASK_PRIO 4

#define JUDEG_RX_TASK_SIZE 256
#define JUDEG_RX_TASK_PRIO 4

#define PC_TX_TASK_SIZE 256
#define PC_TX_TASK_PRIO 4

#define PC_RX_TASK_SIZE 256
#define PC_RX_TASK_PRIO 4

TaskHandle_t start_Task_Handle;
TaskHandle_t gimbal_Task_Handle;
TaskHandle_t chassis_Task_Handle;
TaskHandle_t shoot_Task_Handle;
TaskHandle_t can_msg_send_Task_Handle;
TaskHandle_t mode_switch_Task_Handle;
TaskHandle_t info_get_Task_Handle;
TaskHandle_t detect_Task_Handle;
TaskHandle_t imu_Task_Handle;

TaskHandle_t judge_tx_Task_Handle;
TaskHandle_t judge_rx_Task_Handle;
TaskHandle_t pc_tx_Task_Handle;
TaskHandle_t pc_rx_Task_Handle;

void start_task(void *parm)
{
	taskENTER_CRITICAL();
	
	{
		xTaskCreate((TaskFunction_t)gimbal_task,
							(const char *)"gimbal_task",
							(uint16_t)GIMBAL_TASK_SIZE,
							(void * )NULL,
							(UBaseType_t)GIMBAL_TASK_PRIO,
							(TaskHandle_t *) &gimbal_Task_Handle );
    
              
    xTaskCreate((TaskFunction_t)chassis_task,
							(const char *)"chassis_task",
							(uint16_t)CHASSIS_TASK_SIZE,
							(void * )NULL,
							(UBaseType_t)CHASSIS_TASK_PRIO,
							(TaskHandle_t *) &chassis_Task_Handle );
              
		xTaskCreate((TaskFunction_t)shoot_task,
							(const char *)"shoot_task",
							(uint16_t)SHOOT_TASK_SIZE,
							(void * )NULL,
							(UBaseType_t)SHOOT_TASK_PRIO,
							(TaskHandle_t *) &shoot_Task_Handle );
							
	
  }        

  {
		xTaskCreate((TaskFunction_t)can_msg_send_task,
								(const char *)"can_msg_send_task",
								(uint16_t)CAN_MSG_SEND_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)CAN_MSG_SEND_TASK_PRIO,
								(TaskHandle_t *) &can_msg_send_Task_Handle );
	}
	
	{						
		xTaskCreate((TaskFunction_t)mode_switch_task,
							(const char *)"mode_switch_task",
							(uint16_t)MODE_SWITCH_TASK_SIZE,
							(void * )NULL,
							(UBaseType_t)MODE_SWITCH_TASK_PRIO,
							(TaskHandle_t *) &mode_switch_Task_Handle );
							
		xTaskCreate((TaskFunction_t)info_get_task,
								(const char *)"info_get_task",
								(uint16_t)INFO_GET_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)INFO_GET_TASK_PRIO,
								(TaskHandle_t *) &info_get_Task_Handle );
								
		xTaskCreate((TaskFunction_t)detect_task,
								(const char *)"detect_task",
								(uint16_t)DETECT_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)DETECT_TASK_PRIO,
								(TaskHandle_t *) &detect_Task_Handle );
								
		xTaskCreate((TaskFunction_t)imu_task,
								(const char *)"imu_task",
								(uint16_t)IMU_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)IMU_TASK_PRIO,
								(TaskHandle_t *) &imu_Task_Handle );
	}						
	
  {
    xTaskCreate((TaskFunction_t)judge_tx_task,
								(const char *)"judge_tx_task",
								(uint16_t)JUDEG_TX_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)JUDEG_TX_TASK_PRIO,
								(TaskHandle_t *) &judge_tx_Task_Handle );
                
    xTaskCreate((TaskFunction_t)judge_rx_task,
								(const char *)"judge_rx_task",
								(uint16_t)JUDEG_RX_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)JUDEG_RX_TASK_PRIO,
								(TaskHandle_t *) &judge_rx_Task_Handle );
                
    xTaskCreate((TaskFunction_t)pc_tx_task,
								(const char *)"pc_tx_task",
								(uint16_t)PC_TX_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)PC_TX_TASK_PRIO,
								(TaskHandle_t *) &pc_tx_Task_Handle );
                
    xTaskCreate((TaskFunction_t)pc_rx_task,
								(const char *)"pc_rx_task",
								(uint16_t)PC_RX_TASK_SIZE,
								(void * )NULL,
								(UBaseType_t)PC_RX_TASK_PRIO,
								(TaskHandle_t *) &pc_rx_Task_Handle );
    
  }
  
	vTaskDelete(start_Task_Handle);//删除开始任务（利用任务句柄完成删除操作 删除自身任务也可使用NULL）
	taskEXIT_CRITICAL();						 //退出临界区
}

void TASK_START(void)
{
	xTaskCreate((TaskFunction_t)start_task,
							(const char *)"start_task",
							(uint16_t)START_TASK_SIZE,
							(void * )NULL,
							(UBaseType_t)START_TASK_PRIO,
							(TaskHandle_t *) &start_Task_Handle );
              
}
