#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "pc_rx_data.h"
#include "string.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"

extern TaskHandle_t can_msg_send_Task_Handle;
UBaseType_t shoot_stack_surplus;



extern uint8_t shoot_cmd; //垂直摩擦轮标志位
extern uint8_t shoot_glass_flag; //倍镜标志位
extern uint8_t shoot_image_flag;//图传标志位

extern uint8_t shoot_R3_angle_add;//舵机角度增加标志位
extern uint8_t shoot_R3_angle_sub;//舵机角度减少标志位

uint8_t finish_flag = 1; //倍镜副标志位

	uint8_t shoot_ready; //弹丸是否上膛判断
	uint32_t shoot_last_time; //记录上一次发弹时间
	uint8_t  shoot_R3_dynamic_angle = 49;
//**********************************************************
	/* 摩擦轮转速 */
	uint16_t sixteen_fric_speed_front = 5600;	//	16m/s模式射速
	uint16_t sixteen_fric_speed_rear = 4700;
	/*图传角度*/
	uint16_t shoot_5m_image_angle = 70;
	uint16_t shoot_R3_image_angle = 49;
	/*摩擦轮pid*/
	float fric_pid[3] = {10, 0.001f, 0};
	float trigger_pid[6] = {30,0,0,10,0,0};
	float glass_pid[6] = {60,0,0,10,0,0};
//**********************************************************
shoot_t   shoot;   //水平摩擦轮
trigger_t trig_ref;//垂直摩擦轮
glass_t glass;		//倍镜

/*倍镜开关控制，具有局限性，需要机械限位，并且上电其实位置需为左下底部*/	
static void magnifying_glass_handler(void)
{
	//开
	if(shoot_glass_flag && finish_flag)
	{
		//glass.spd_ref = 3000;
		glass.angle_ref = moto_glass.total_angle + 205;
		finish_flag = 0;
	}

		//关
	if(!shoot_glass_flag && !finish_flag)
	{
		//glass.spd_ref = -3000;
		glass.angle_ref = moto_glass.total_angle - 205;
		finish_flag = 1;
	}
		
	pid_calc(&pid_glass,moto_glass.total_angle,glass.angle_ref);
	glass.spd_ref = pid_glass.out;
		
	pid_calc(&pid_glass_spd, moto_glass.speed_rpm, glass.spd_ref);
	glb_cur.glass_cur =pid_glass_spd.out;
}

//垂直摩擦轮控制	
static void shoot_bullet_handler(void)
{
	if(shoot_cmd )
	{
		trig_ref.spd_ref = -9000;//-3140;//3140/36=87.22rpm//修改限位转速
		trig_ref.angle_ref = moto_trigger.total_angle - 90;
		shoot_last_time = HAL_GetTick();
	}
	if(moto_trigger.total_angle <= trig_ref.angle_ref)//未转到给定位置摩擦轮便一直有速度
	{
		pid_calc(&pid_trigger_angle,moto_trigger.total_angle,trig_ref.angle_ref);
		trig_ref.spd_ref = pid_trigger_angle.out;
	}
	
	if(HAL_GetTick()-shoot_last_time>65)//偶尔编码位置丢失摩擦轮会一直转，一段时间后速度给0
		trig_ref.spd_ref = 0;
	
	pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, trig_ref.spd_ref);
	glb_cur.fric_cur[VERTICAL_FRIC] = pid_trigger_spd.out;
}

//42mm轻触开关检测->控制底盘拨盘旋转与垂直摩擦轮通断
void shoot_ready_flag_handle(void)
{
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11) == Bit_SET)
		{
			GPIO_SetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11);GPIO_ResetBits(GPIOH,GPIO_Pin_12);//shoot_ready = 1 亮蓝灯+绿灯 = 青色
			shoot_ready = 1;
		}
		else if (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11) == Bit_RESET)
		{
			shoot_ready = 0;
			GPIO_SetBits(GPIOH,GPIO_Pin_11);GPIO_ResetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_12);//IMU初始化完成亮绿灯11
		}
}

static void servor_image_handle(void)
{
	uint16_t pulse_width;
	
	if(shoot_image_flag != 1)
	{
		 pulse_width = (uint16_t)(0.5 * 20000 / 20 + shoot_5m_image_angle * 20000 / 20 / 90);
	}
    else if(shoot_image_flag != 0)
	{

		 pulse_width = (uint16_t)(0.5 * 20000 / 20 + shoot_R3_image_angle * 20000 / 20 / 90);
	}
    // 设置定时器的捕获比较寄存器来产生所需的PWM占空比
    TIM_SetCompare3(TIM8, pulse_width);
}

void shoot_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_SHOOT_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
   {
	if(Signal & INFO_GET_SHOOT_SIGNAL)
	{
        if(1)
        {          
          /*水平摩擦轮*/
          for(int i=0;i<4;i++)
          {
            PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 14500, 800, DONE); 
          }
					
			/*垂直摩擦轮*/
			PID_Struct_Init(&pid_trigger_angle, trigger_pid[0], trigger_pid[1], trigger_pid[2],5000, 100, DONE);
			PID_Struct_Init(&pid_trigger_spd, trigger_pid[3], trigger_pid[4], trigger_pid[5],8000, 500, DONE);
					
          	/*倍镜*/
			PID_Struct_Init(&pid_glass, glass_pid[0], glass_pid[1], glass_pid[2],2000, 100, DONE);
			PID_Struct_Init(&pid_glass_spd, glass_pid[3], glass_pid[4], glass_pid[5],4000, 500, DONE);
					
          	//42mm轻触开关检测->控制底盘拨盘旋转与垂直摩擦轮通断
			shoot_ready_flag_handle();
			
			shoot_bullet_handler();				//垂直摩擦轮发射控制
			magnifying_glass_handler();   //倍镜控制
					
			shoot_para_ctrl();						// 射击速度选择
			fric_wheel_ctrl();							// 判断是否启动摩擦轮
			servor_image_handle();						//R3狙击				
          if(!shoot.para_mode)
          {
			shoot.fric_wheel_spd_front_ref = 0;
			shoot.fric_wheel_spd_rear_ref = 0;

            pid_trigger_spd.out = 0;
            trig_ref.angle_ref = moto_trigger.total_angle; //记录当前垂直摩擦轮电机编码位，令垂直摩擦轮保持不动
			pid_glass_spd.out = 0;
			shoot_last_time = 0;
		for(int i=0;i<4;i++)
		  {
			pid_fric[i].iout = 0;
			  pid_fric[i].iterm = 0;
			   pid_fric[i].out = 0;
		  }
          }
          
          get_last_shoot_mode();
        }
        else
        {
       //   pid_trigger_spd.out = 0;

//			shoot.fric_wheel_spd_front_ref = 0;
//			shoot.fric_wheel_spd_rear_ref = 0;
        }
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) SHOT_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
      }
    }
			
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
		
} 


/*获取上一次的射击模式*/
void get_last_shoot_mode(void)
{
	shoot.last_para_mode = shoot.para_mode;
}

/** @brief 		发射模式转速选择
**  @date 	    2024/1/29
**  @attention  24赛季开始为恒定16m/s射速
**/
/*摩擦轮转速赋值，可在这里写动态射速*/
static void shoot_para_ctrl(void)
{

	shoot.fric_wheel_spd_front_ref = sixteen_fric_speed_front;		
	shoot.fric_wheel_spd_rear_ref = sixteen_fric_speed_rear;
}

/*摩擦轮控制*/
static void fric_wheel_ctrl(void)
{
  
	if (shoot.para_mode != SHOOTSTOP_MODE)
	{
		turn_on_friction_wheel(-shoot.fric_wheel_spd_front_ref, shoot.fric_wheel_spd_rear_ref);
	}
	else if (shoot.para_mode == SHOOTSTOP_MODE)
	{
		turn_off_friction_wheel();
	}
}

/*打开水平摩擦轮*/
static void turn_on_friction_wheel(int16_t lspd,int16_t rspd)
{
	//右前
	pid_calc(&pid_fric[RIGHT_FRIC_FRONT], moto_fric[RIGHT_FRIC_FRONT].speed_rpm, rspd);
	glb_cur.fric_cur[RIGHT_FRIC_FRONT] = pid_fric[RIGHT_FRIC_FRONT].out;
	//左前
	pid_calc(&pid_fric[LEFT_FRIC_FRONT], moto_fric[LEFT_FRIC_FRONT].speed_rpm, lspd);
	glb_cur.fric_cur[LEFT_FRIC_FRONT] = pid_fric[LEFT_FRIC_FRONT].out;
	//左后
	pid_calc(&pid_fric[LEFT_FRIC_REAR], moto_fric[LEFT_FRIC_REAR].speed_rpm, lspd);
	glb_cur.fric_cur[LEFT_FRIC_REAR] = pid_fric[LEFT_FRIC_REAR].out;
	//右后
	pid_calc(&pid_fric[RIGHT_FRIC_REAR], moto_fric[RIGHT_FRIC_REAR].speed_rpm, rspd);
	glb_cur.fric_cur[RIGHT_FRIC_REAR] = pid_fric[RIGHT_FRIC_REAR].out;



}

/*关闭水平摩擦轮*/
static void turn_off_friction_wheel(void)
{
	//右前
	pid_calc(&pid_fric[RIGHT_FRIC_FRONT], moto_fric[RIGHT_FRIC_FRONT].speed_rpm, 0);
	glb_cur.fric_cur[RIGHT_FRIC_FRONT] = pid_fric[RIGHT_FRIC_FRONT].out;
	//左前
	pid_calc(&pid_fric[LEFT_FRIC_FRONT], moto_fric[LEFT_FRIC_FRONT].speed_rpm, 0);
	glb_cur.fric_cur[LEFT_FRIC_FRONT] = pid_fric[LEFT_FRIC_FRONT].out;
	//左后
	pid_calc(&pid_fric[LEFT_FRIC_REAR], moto_fric[LEFT_FRIC_REAR].speed_rpm, 0);
	glb_cur.fric_cur[LEFT_FRIC_REAR] = pid_fric[LEFT_FRIC_REAR].out;
	//右后
	pid_calc(&pid_fric[RIGHT_FRIC_REAR], moto_fric[RIGHT_FRIC_REAR].speed_rpm, 0);
	glb_cur.fric_cur[RIGHT_FRIC_REAR] = pid_fric[RIGHT_FRIC_REAR].out;
	

}

/*发射机构初始化*/
void shoot_param_init(void)
{
  memset(&shoot, 0, sizeof(shoot_t));
  
	shoot.para_mode			 = SHOOTMAD_MODE;
  
  memset(&trig_ref, 0, sizeof(trigger_t));
  memset(&glass, 0, sizeof(glass_t));
  
  /*水平摩擦轮*/
  for(int i=0;i<4;i++)
  {
    PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 14500, 500, INIT); 
  }
	/*垂直摩擦轮*/
	PID_Struct_Init(&pid_trigger_angle, trigger_pid[0], trigger_pid[1], trigger_pid[2],5000, 100, DONE);
	PID_Struct_Init(&pid_trigger_spd, trigger_pid[3], trigger_pid[4], trigger_pid[5],8000, 500, DONE);  
		  for(int i=0;i<4;i++)
		  {
			pid_fric[i].iout = 0;
			  pid_fric[i].iterm = 0;
		  }
}

