#include "bsp_can.h"
#include "detect_task.h"
#include "sys_config.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "comm_task.h"
#include "chassis_task.h"
#include "pc_tx_data.h"
#include "math.h"
#include "shoot_task.h"
#include "gimbal_task.h"

CanRxMsg rx1_message;
CanRxMsg rx2_message;

moto_measure_t moto_chassis[4];//底盘
moto_measure_t moto_pit; //pitch
moto_measure_t moto_yaw; //yaw
moto_measure_t moto_loader[2]; //拨盘
//
float pit_angle;
float *p_angle = &pit_angle;
float distance;
float *p_distance = &distance;



uint8_t shoot_ready;//判断是否上膛
uint8_t shoot_cmd;//垂直摩擦轮发射标志位
uint8_t gimbal_follow_chassis;//云台模式选择标志位
extern uint8_t left_offset_cmd;//扩展装甲板(左)
extern uint8_t right_offset_cmd;//扩展装甲板(右)


static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan,CanRxMsg *message)
{
	if(_hcan == CAN1)
	{
		switch(message->StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
        //处理电机ID号
        i = message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], message) : encoder_data_handler(&moto_chassis[i], message);
        //WDOG
        err_detector_hook(CHASSIS_M1_OFFLINE + i);
       
			}break;
			
			case CAN_TRIGGER_MOTOR_ID1:
			{
				moto_loader[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_loader[0], message) : encoder_data_handler(&moto_loader[0], message);
        err_detector_hook(TRIGGER_MOTO_OFFLINE_FRONT);  
			}break;
			
			case CAN_TRIGGER_MOTOR_ID2:
			{
				moto_loader[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_loader[1], message) : encoder_data_handler(&moto_loader[1], message);
        err_detector_hook(TRIGGER_MOTO_OFFLINE_REAR);  
			}break;
			case CAN_SUPER_CAP_ID:
			{
					chassis.CapData[0] = (float)((rx1_message.Data[1] << 8 | rx1_message.Data[0])/100.f);		//输入电压
					chassis.CapData[1] = (float)((rx1_message.Data[3] << 8 | rx1_message.Data[2])/100.f);		//电容电压
					chassis.CapData[2] = (float)((rx1_message.Data[5] << 8 | rx1_message.Data[4])/100.f);		//输入电流
					chassis.CapData[3] = (float)((rx1_message.Data[7] << 8 | rx1_message.Data[6])/100.f);		//设定功率	
			}break;
			
			default:
			{
			}break;
		}
	}
	else
	{
		switch(message->StdId)
    {	
			case CAN_YAW_MOTOR_ID: //接收yaw电机
			{
				encoder_data_handler(&moto_yaw, message);
				err_detector_hook(GIMBAL_YAW_OFFLINE);
			}break;
			
			case GIMBAL_MASTER_ID: //接收测距距离，pit相对角，轻触开关
			{
				uint8_t *px = rx2_message.Data;
        for(int i = 0; i < 4; i++)
        {
          *((uint8_t*)p_angle + i) = *(px + i);//pit+轻触开关
        }
				for(int i = 0; i < 4; i++)
        {
          *((uint8_t*)p_distance + i) = *(px + i + 4);//距离
        }	
					
				pit_encode(&pit_angle); //pit百位数位轻触开关有效位，要解码得到pit和轻触开关的标志位
			  gimbal.sensor.pit_relative_angle = pit_angle;
				
      }break;

      default:
      {
      }break;
    }
	}
}

//读取电机转速，角度，编码位置等等
void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	if(ptr == &moto_loader[0]||ptr == &moto_loader[1])
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO*19);
	else
		ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
	
	ptr->speed_rpm     = (int16_t)(message->Data[2] << 8 | message->Data[3]);
  ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]);

}


/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}


/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */

//发送底盘电流
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

//发送拨盘电流
void send_trig_cur1(int16_t iq7,int16_t iq8)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_TRIG_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[4] = iq7 >> 8;
    TxMessage.Data[5] = iq7;
		TxMessage.Data[6] = iq8 >> 8;
    TxMessage.Data[7] = iq8;

    CAN_Transmit(LOADER_CAN1, &TxMessage);	
}

//发给功率控制板限制功率
void send_cap_power_can(uint16_t tempower)
{
	  CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CAP_POWER_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = tempower >> 8;
    TxMessage.Data[1] = tempower;

    CAN_Transmit(SUPER_CAP_CAN, &TxMessage);	
	
}

/**
函数名：send_servant_angle
作用：发送由遥控器接收的给定角度给云台开发板
参数：yaw_ref——yaw给定，pit_ref——pit给定
**/
void send_servant_angle(float yaw_ref, float pit_ref)
{	
	unsigned char *p1;
	p1 = (unsigned char *)&yaw_ref;
	unsigned char *p2;
	p2 = (unsigned char *)&pit_ref;

  CanTxMsg TxMessage;
  TxMessage.StdId = CAN_ANGLE_REF_ID;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = 0x08;
  TxMessage.Data[0] = *p1;
  TxMessage.Data[1] = *(p1+1);
  TxMessage.Data[2] = *(p1+2);
  TxMessage.Data[3] = *(p1+3);
  TxMessage.Data[4] = *p2;
  TxMessage.Data[5] = *(p2+1);
  TxMessage.Data[6] = *(p2+2);
  TxMessage.Data[7] = *(p2+3);
 
  CAN_Transmit(SERVANT_CAN, &TxMessage); //can2
}
/**
函数名：send_servant_mode
作用：发送遥控器拨杆，摩擦轮模式（停止，10m，16m）,云台yaw是否有输入(抑制温漂用)，云台模式，发射命令，敌方颜色给云台开发板
参数：sw1——拨杆1，sw2——拨杆2，firc_flag——摩擦轮模式，action_flag——云台yaw是否输入
（云台模式，发射命令，敌方颜色不包括在参数里，而是函数直接赋值——写的有点不规范）
**/
void send_servant_mode(char sw1, char sw2, char firc_flag, char action_flag)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_MODE_ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = sw1; //拨杆1
	TxMessage.Data[1] = sw2; //拨杆2
	TxMessage.Data[2] = firc_flag;//摩擦轮模式
	TxMessage.Data[3] = action_flag;//云台yaw是否有输入(抑制温漂用)
	TxMessage.Data[4] = gimbal_follow_chassis;//云台模式
	TxMessage.Data[5] = shoot_cmd;//发射命令
	TxMessage.Data[6] = pc_send_mesg.robot_color;//敌方颜色
	
	CAN_Transmit(SERVANT_CAN, &TxMessage);//CAN2
}

/**
** 函数名：send_bullet_shoot_cmd
** 作用：CV按键，倍镜开关给云台开发板
**/
void send_bullet_shoot_cmd(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_SHOOT_CMD;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;

	TxMessage.Data[1] = left_offset_cmd;//C键
	TxMessage.Data[2] = right_offset_cmd;//V键
	TxMessage.Data[3] = shoot.shoot_glass_flag;//倍镜
	TxMessage.Data[4] = shoot.shoot_image_flag;//图传
	CAN_Transmit(CAN2, &TxMessage);//CAN2
}

void pit_encode(float *angle_flag)//读取云台pit解码 百位数为轻触开关有效位，十位个位小数为pit角度，正负号不变
{
	if(*angle_flag >= 0) //100.0  000.0
	{
		if(*angle_flag - 100 >= 0)	shoot_ready = 1;
		else shoot_ready = 0;
		if(shoot_ready == 1) *angle_flag -= 100;
	}
	else     //-100.0  -000.0
	{
		if(*angle_flag + 100 <= 0)	shoot_ready = 1;
		else shoot_ready = 0;
		if(shoot_ready == 1)*angle_flag += 100;
	}
}

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        STD_CAN_RxCpltCallback(CAN1,&rx1_message);
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        STD_CAN_RxCpltCallback(CAN2,&rx2_message);
    }
}


