#include "bsp_can.h"
#include "detect_task.h"
#include "sys_config.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "comm_task.h"
#include "chassis_task.h"
#include "rc.h"
#include "shoot_task.h"
#include "pc_tx_data.h"


CanRxMsg rx1_message;
CanRxMsg rx2_message;

moto_measure_t moto_pit;//pitch
moto_measure_t moto_yaw;//yaw
moto_measure_t moto_trigger;//垂直摩擦轮
moto_measure_t moto_fric[4];//水平摩擦轮
moto_measure_t moto_glass;//倍镜

//**********pitch/yaw期望遥控器指针解码******
float angle;
float *p_angle = &angle;

float angle_yaw;//rc yaw 期望给定
float *yaw_angle_t = &angle_yaw;

float angle_pit;//rc pitch 期望给定
float *pit_angle_t = &angle_pit;
//***************************************
gimbal_state_t gimbal_state; //遥控器yaw状态获取
uint8_t gimbal_follow_chassis; //云台随底盘模式变化标志位

enemy_color_t enemy_color; //敌方颜色获取
uint8_t shoot_cmd;				//垂直摩擦轮发射标志位
uint8_t shoot_glass_flag;//倍镜标志位
uint8_t shoot_image_flag = 0;//图传标志位
uint8_t left_offset_cmd = 0;	//扩展装甲板（左）
uint8_t	right_offset_cmd = 0; //扩展装甲板（右）
float shoot_42mm_speed;//42mm弹丸初速度
uint8_t shoot_R3_angle_add;
uint8_t shoot_R3_angle_sub;
static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan,CanRxMsg *message)
{
	if(_hcan == CAN1)//can1中断
	{
		switch(message->StdId)
		{
			case CAN_YAW_MOTOR_ID: //yaw电机
			{
				encoder_data_handler(&moto_yaw, message);
				err_detector_hook(GIMBAL_YAW_OFFLINE);
			}break;
			
			case CAN_GLASS_ID:
			{
				 moto_glass.msg_cnt++ <= 50 ? get_moto_offset(&moto_glass, message) : encoder_data_handler(&moto_glass, message);
        err_detector_hook(GLASS_MOTO_OFFLINE);	
			}break;
			
      case CAN_TRIG_ID:
			{
        moto_trigger.msg_cnt++ <= 50 ? get_moto_offset(&moto_trigger, message) : encoder_data_handler(&moto_trigger, message);
        err_detector_hook(TRIGGER_MOTO_OFFLINE);				
			}break;
			case CAN_ANGLE_REF_ID: //底盘遥控器接收机发送过来的角度
			{
				uint8_t *px = rx1_message.Data;
        for(int i = 0; i < 4; i++)
        {
          *((uint8_t*)yaw_angle_t + i) = *(px + i); //yaw
        }
				for(int i = 0; i < 4; i++)
        {
          *((uint8_t*)pit_angle_t + i) = *(px + i + 4); //pit
        }	
			}break;
			
			case CAN_MODE_ID: //遥控器模式
			{
				rc.sw1 = rx1_message.Data[0]; //拨杆1 
				rc.sw2 = rx1_message.Data[1]; //拨杆2
				shoot.para_mode = rx1_message.Data[2]; //发射使能（转速选择）
				gimbal_state = (gimbal_state_t)rx1_message.Data[3];    //判断云台yaw是否有遥控或者键盘的给定
				gimbal_follow_chassis = rx1_message.Data[4]; //云台模式——自瞄，正常，小陀螺，分离
				shoot_cmd = rx1_message.Data[5];//垂直摩擦轮发射标志位
				enemy_color = (enemy_color_t)rx1_message.Data[6]; //敌方颜色，由裁判系统读取
				//shoot_42mm_speed = (float)rx1_message.Data[7];
			}break;
			
			case CAN_SHOOT_CMD_ID://底盘信息传输
			{
				
				shoot_R3_angle_add = rx1_message.Data[1];//C键
				shoot_R3_angle_sub = rx1_message.Data[2];//V键
				shoot_glass_flag = rx1_message.Data[3];//X倍镜按键
				shoot_image_flag = rx1_message.Data[4];//F图传按键
			
			}break;
      default:
      {
      }break;			
		}
	}
	else //can2中断
	{
		switch(message->StdId)
    {
			
			/***摩擦轮***/
      case CAN_FRIC_M1_ID:
      case CAN_FRIC_M2_ID:
      case CAN_FRIC_M3_ID:
      case CAN_FRIC_M4_ID:
      {
        static uint8_t i = 0;
        //处理电机ID号
        i = message->StdId - CAN_FRIC_M1_ID;
        //处理电机数据宏函数
        moto_fric[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric[i], message) : encoder_data_handler(&moto_fric[i], message);
        err_detector_hook(FRI_MOTO1_OFFLINE + i);
      }break;
      
			case CAN_PIT_MOTOR_ID:
			{
				encoder_data_handler(&moto_pit, message);
				err_detector_hook(GIMBAL_PIT_OFFLINE);
			}break;
			

			    
      default:
      {
      }break;
    }
	}
}


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
	if(ptr == &moto_trigger)
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO*36);
		else if(ptr == &moto_pit)
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO*3.4f);
		
		
	else if(ptr == &moto_glass)
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO*36);
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

//**************************************************************************************//
//yaw
void send_gimbal_yaw_cur(int16_t yaw_iq) //!!!
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x2ff;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;

    TxMessage.Data[2] = yaw_iq >> 8; //ID 6
    TxMessage.Data[3] = yaw_iq;
	
		CAN_Transmit(CAN1, &TxMessage);
}
//pitch
void send_gimbal_pit_cur( int16_t pit_iq) //!!!
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x2ff;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[4] = pit_iq >> 8; //ID 7
    TxMessage.Data[5] = pit_iq;
	
		CAN_Transmit(CAN2, &TxMessage);
}

//fric
void send_fric_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) //!!!
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x200;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = iq1 >> 8;//摩擦轮 右前 ID 1
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;//摩擦轮  左前 ID 2
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;//摩擦轮 左后 ID 3
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;//摩擦轮 右后 ID 4
    TxMessage.Data[7] = iq4;

    CAN_Transmit(CAN2, &TxMessage);
}
void send_vertical_fric_cur(int16_t iq1) //!!!
{
        CanTxMsg TxMessage;
    TxMessage.StdId = 0x1ff;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[6] = iq1 >> 8;//垂直摩擦轮 CAN1 ID 1
    TxMessage.Data[7] = iq1;

    CAN_Transmit(CAN1, &TxMessage);
}
//倍镜
void send_magnifying_glass_cur(int16_t glass_iq)
{
		CanTxMsg TxMessage;
    TxMessage.StdId = 0x200;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[4] = glass_iq >> 8; //倍镜 ID 1
    TxMessage.Data[5] = glass_iq;
	
		CAN_Transmit(CAN1, &TxMessage);

}

/*
函数名：send_to_chassis
参数：
pit——pit相对角度，dis——测距距离，shoot_ready_flag——轻触开关是否触碰
*/
void send_to_chassis(float pit,float dis,uint8_t shoot_ready_flag)
{
	CanTxMsg TxMessage;
	float pit_shoot;
	unsigned char *p1;
	p1 = (unsigned char *)&pit_shoot;//p1——包含pit和shoot_ready_flag信息
	unsigned char *p2;
	p2 = (unsigned char *)&dis;//p2——包含dis信息
	/**由于字节长度受限，把pit相对角与轻触开关合起来（pit不会超过100度，100作为轻触开关标志位）
	轻触开关碰到，pit 10度时，发送 110；轻触开关没碰到，pit 10度，发送10；
	轻触开关碰到，pit -10度，发送-110；轻触开关没碰到，pit -10，发送-10**/
	if(shoot_ready_flag) 
	{	
		if(pit >= 0)pit_shoot = pit +100;
		else if(pit < 0)pit_shoot = pit -100;
	}
	else
		pit_shoot = pit;
	/****/

  TxMessage.StdId = CAN_TO_CHASSIS_ID; //发送IDCAN_TO_CHASSIS_ID
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

  CAN_Transmit(CAN1, &TxMessage);	
		
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


