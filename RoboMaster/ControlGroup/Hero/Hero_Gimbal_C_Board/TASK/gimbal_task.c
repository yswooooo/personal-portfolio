#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"
#include "info_get_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "chassis_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()函数
#include "math.h"   //fabs()函数
#include "ladrc.h"

UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

gimbal_t gimbal; //云台电机pitch/yaw控制所需变量
ramp_t pit_ramp;//pitch归中斜坡函数
ramp_t yaw_ramp;//yaw归中斜坡函数
extern float angle_yaw;//yaw角度期望
extern float angle_pit;//pitch角度期望
extern gimbal_state_t gimbal_state;//yaw当前状态是否有输入
extern signed char i_temp;//用于记录yaw轴转了多少圈

/* 归中pid参数 */
float pit_pid_init[6] = {70,1,150,70,0,0};//pit轴pid
float yaw_pid_init[6] = {70,1,150,70,0,0};//yaw轴pid

/* 正常模式pid参数 */
float pit_pid[6] = {70,0.7,220,60,0,0};//pit轴pid60,163
float yaw_pid[6] = {45,0.2,350,40,0,0};//yaw轴pid

/* 自瞄模式pid参数 */
float pit_track_pid[6] = {25,0.05,175,70,0,0};
float yaw_track_pid[6] = {29,0.05,150,220,0,0};

//!!!电机方向与电机安装方向有关！！！ 
float pit_dir = -1.f;  //目前是在右边，底部向<---
float yaw_dir = -1.f; //目前是反装,底部向↑

/* 前馈控制器，使用时更改控制器宏定义 */
Easy_FFC_t yaw_easy_ffc;
Easy_FFC_t pit_easy_ffc;
float pit_track_ctrl;
float yaw_track_ctrl;
float pit_pid_ctrl;
float yaw_pid_ctrl;

FFC p_ffc;
FFC y_ffc;
/* PITCH限位宏 */
#define PIT_ANGLE_MAX      40
#define PIT_ANGLE_MIN      -18
/* YAW限位宏 */
#define YAW_ANGLE_MAX      50
#define YAW_ANGLE_MIN      -50

/*快速归中宏定义*/
#define FAST_CALIBRATE 0
/*控制器选择宏定义*/
#define ANGLE_PID 0
#define SPEED_PID 0
#define FUZZY_SPEED_PID 1
#define EASY_FFC_PID 1

void gimbal_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  while(1)
	{
		STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
	{
			
	 if(Signal & INFO_GET_GIMBAL_SIGNAL)
	  {

		if(gimbal_mode == GIMBAL_INIT)//yaw
		{
			/* pit 轴电机的PID参数 */
			PID_Struct_Init(&pid_pit, pit_pid_init[0], pit_pid_init[1], pit_pid_init[2], 23000, 1000, DONE);
			PID_Struct_Init(&pid_pit_spd, pit_pid_init[3], pit_pid_init[4], pit_pid_init[5], 23000, 10000, DONE);

			/* yaw 轴电机的PID参数 */
			PID_Struct_Init(&pid_yaw, yaw_pid_init[0], yaw_pid_init[1], yaw_pid_init[2], 23000, 1000, DONE); 
			PID_Struct_Init(&pid_yaw_spd, yaw_pid_init[3] , yaw_pid_init[4], yaw_pid_init[5], 23000, 10000, DONE);  
		}
		else if(gimbal_mode != GIMBAL_TRACK_ARMOR)
		{
			/* pit 轴电机的PID参数 */
			PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 23000, 1000, DONE);
			PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 23000, 1000, DONE);

			/* yaw 轴电机的PID参数 */
			PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 23000, 1000, DONE); 
			PID_Struct_Init(&pid_yaw_spd, yaw_pid[3] , yaw_pid[4], yaw_pid[5], 23000, 1000, DONE);  
		}
		
		else if(gimbal_mode == GIMBAL_TRACK_ARMOR)
		{
			/* pit 轴电机的PID参数 */
			PID_Struct_Init(&pid_pit, pit_track_pid[0],pit_track_pid[1], pit_track_pid[2], 23000, 800, DONE);
			PID_Struct_Init(&pid_pit_spd, pit_track_pid[3], pit_track_pid[4], pit_track_pid[5], 23000, 4000, DONE);

			/* yaw 轴电机的PID参数 */
			PID_Struct_Init(&pid_yaw, yaw_track_pid[0], yaw_track_pid[1], yaw_track_pid[2], 23000, 800, DONE); 
			PID_Struct_Init(&pid_yaw_spd, yaw_track_pid[3] , yaw_track_pid[4], yaw_track_pid[5], 23000, 4000, DONE);  					
		}
        
        if(gimbal_mode != GIMBAL_RELEASE)//当云台非RELEASE
        {
          switch(gimbal_mode)
          {
			 /*云台底盘归中模式*/
            case GIMBAL_INIT:
            {
				if(last_gimbal_mode != GIMBAL_INIT )
				{
					PID_Clear(&pid_pit);
					PID_Clear(&pid_yaw);
					PID_Clear(&pid_pit_spd);
					PID_Clear(&pid_yaw_spd);
				}
              init_mode_handler(); //云台回中
            }break;
            /*云台底盘跟随模式*/
            case GIMBAL_NORMAL_MODE:
            {
				if(last_gimbal_mode != GIMBAL_NORMAL_MODE )
				{
					PID_Clear(&pid_pit);
					PID_Clear(&pid_yaw);
					PID_Clear(&pid_pit_spd);
					PID_Clear(&pid_yaw_spd);
				}
              nomarl_handler();
            }break;
            /*云台底盘分离模式*/
            case GIMBAL_SEPARATE_MODE:
            {
				if(last_gimbal_mode != GIMBAL_SEPARATE_MODE )
				{
					PID_Clear(&pid_pit);
					PID_Clear(&pid_yaw);
					PID_Clear(&pid_pit_spd);
					PID_Clear(&pid_yaw_spd);
				}
              separate_handler();
            }break;
            /*小陀螺模式*/
            case GIMBAL_DODGE_MODE:
            {
				if(last_gimbal_mode != GIMBAL_DODGE_MODE )
				{
					PID_Clear(&pid_pit);
					PID_Clear(&pid_yaw);
					PID_Clear(&pid_pit_spd);
					PID_Clear(&pid_yaw_spd);
				}
              dodge_handler();
            }break;
         
            /*自瞄模式*/
            case GIMBAL_TRACK_ARMOR:
            {
				if(last_gimbal_mode != GIMBAL_TRACK_ARMOR )
				{
					PID_Clear(&pid_pit);
					PID_Clear(&pid_yaw);
					PID_Clear(&pid_pit_spd);
					PID_Clear(&pid_yaw_spd);
				}
              track_aimor_handler();
            }break;
            
            default:
            {
            }break;
          }
        }
        else//gimbal_mode == REALEASE
        {
          memset(glb_cur.gimbal_cur,0,sizeof(glb_cur.gimbal_cur));//云台电机无力
          gimbal.state = GIMBAL_INIT_NEVER;//云台预备归中
        }
        //PID角度环计算
        pid_calc(&pid_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
        pid_calc(&pid_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);

        //PID角度环输出值作为速度环给定值——串级控制
		#if EASY_FFC_PID //速度环前馈期望
		{
			Easy_FFC_Calc(&yaw_easy_ffc, yaw_easy_ffc.Kc, gimbal.pid.yaw_angle_ref);
			Easy_FFC_Calc(&pit_easy_ffc, pit_easy_ffc.Kc, gimbal.pid.pit_angle_ref);
			
			gimbal.pid.yaw_spd_ref = pid_yaw.out + yaw_easy_ffc.out;
        	gimbal.pid.pit_spd_ref = pid_pit.out + pit_easy_ffc.out;
		}
		#elif ANGLE_PID  //普通速度环期望
		{
        	gimbal.pid.yaw_spd_ref = pid_yaw.out;
        	gimbal.pid.pit_spd_ref = pid_pit.out;
		}
		#endif 
		
        //PID速度环反馈给定
        gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;
        gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;
		
		//模糊pid
		#if FUZZY_SPEED_PID //模糊pid计算速度环输出值
        {
			//PID速度环计算——模糊PID控制器
			fuzzy_pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
			fuzzy_pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
		}
		#elif SPEED_PID
		{
			//PID速度环计算——普通PID控制器
			pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
			pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
		} 
		#endif

        /*自瞄模式pit/yaw转矩电流*/
        pit_track_ctrl =  pid_pit_spd.out;//getFeedforwardControl(&p_ffc, gimbal.pid.pit_angle_ref) +pid_pit_spd.out;//前馈
        yaw_track_ctrl =  pid_yaw_spd.out;//getFeedforwardControl(&y_ffc, gimbal.pid.yaw_angle_ref) + pid_yaw_spd.out;

		/*正常模式pit/yaw转矩电流*/
		pit_pid_ctrl = pid_pit_spd.out;
		yaw_pid_ctrl = pid_yaw_spd.out;

        if(gimbal_is_controllable())//云台电机未掉线，则将PID计算值输出
        {
          if(gimbal_mode == GIMBAL_TRACK_ARMOR)
          {
            glb_cur.gimbal_cur[YAW] = yaw_dir * yaw_track_ctrl;
            glb_cur.gimbal_cur[PITCH] = pit_dir * pit_track_ctrl;
          }
          else
          {
            glb_cur.gimbal_cur[YAW] = yaw_dir * yaw_pid_ctrl;
			glb_cur.gimbal_cur[PITCH] = pit_dir * pit_pid_ctrl;
          }
        }
        else
        {
          memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
          gimbal_mode = GIMBAL_RELEASE;//
          pid_trigger_angle.iout = 0;
        }

		get_last_mode();//获取上一次的云台模式 last_gimbal_mode

        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                            (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
      }
    }
    
    gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
  
}


/*云台初始化*/
void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));
  
  gimbal.state = GIMBAL_INIT_NEVER; //开发板刚上电云台要进入归中
  
  ramp_init(&pit_ramp, 1000);
  ramp_init(&yaw_ramp, 1000);

	/* pit 轴电机的PID参数 */
  PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 20000, 500, INIT);
  PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 20000, 4000, INIT);

  /* yaw 轴电机的PID参数 */
  PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 20000, 500, INIT); 
  PID_Struct_Init(&pid_yaw_spd, yaw_pid[3] , yaw_pid[4], yaw_pid[5], 20000, 15000, INIT);  

    /*前馈控制器初始化*/
	Easy_FFC_Init(&pit_easy_ffc,600);
	Easy_FFC_Init(&yaw_easy_ffc,450);
	/* FFC */
	initFeedforwardParam(&p_ffc,210,20);
	initFeedforwardParam(&y_ffc,210,20);
	
}

/* 归中模式，先归PIT再归YAW */
static void init_mode_handler(void)
{
	#if FAST_CALIBRATE
	{
		gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
		gimbal.pid.pit_angle_ref = 0;//快速归中
	}
	#else
	{
		/* PIT轴归中 */
		gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
		gimbal.pid.pit_angle_ref = gimbal.sensor.pit_relative_angle * (1 - ramp_calc(&pit_ramp));//用斜坡函数使给定慢慢变为0，缓慢归中
	}
	#endif
  /* 保持YAW轴不动 */
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
  gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
	//NOTE:为什么没有完成归中云台会一直进行归中操作呢，因为有if判断
	/* 当pitch归中完毕 */
  if(gimbal.pid.pit_angle_fdb >= -3.0f && gimbal.pid.pit_angle_fdb <= 3.0f)
  {
	if(FAST_CALIBRATE)
    	gimbal.pid.yaw_angle_ref = 0;//快速归中
	else
		/*  偏航角轴归中*/
		gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));//用斜坡函数使给定慢慢变为0，缓慢归中
	
    if (gimbal.pid.yaw_angle_fdb >= -1.5f && gimbal.pid.yaw_angle_fdb <= 1.5f)
    {
		gimbal.pid.pit_angle_ref = 0;
		gimbal.pid.yaw_angle_ref = 0;

		gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//归中完毕，记录陀螺仪角度刷新零轴

		gimbal.state = GIMBAL_INIT_DONE;//NOTE:仅作为标志位，代码没有用此flag的任何判断，意味着归中完成，
    }
  }

}

/*判断yaw轴是否有输入*/
static gimbal_state_t remote_is_action(void)
{
	return gimbal_state; //该值从底盘通过CAN2传输上来
}

//正常模式
uint8_t input_flag;
uint32_t no_action_time;
uint32_t debug_time = 1500;//留给底盘的正常模式校准归中时间

static void nomarl_handler(void)
{ 
  if(last_gimbal_mode != GIMBAL_NORMAL_MODE)//一般接下来会跳转到 yaw轴遥控器无输入，且等待延时后，遥控器输入时
  {
    gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//记录yaw轴陀螺仪位置,刷新零轴
		no_action_time = HAL_GetTick();//赋值为后续得到delta时间做记录点
  }
	else
	{
		//YAW轴
		gimbal.state = remote_is_action(); //判断yaw轴是否有输入
		if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)//有运动趋势，但是不会运动，如动！
		{
			gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//记录yaw轴陀螺仪位置,刷新零轴，用于固定云台，这里动之前会以此位置为Zero point作为运动的反馈offset
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;//以（陀螺仪角度-记录角度的偏移量）为反馈值
			gimbal.pid.yaw_angle_ref = 0 ;//给定清零，很重要
			no_action_time = HAL_GetTick();
		}
		else if((gimbal.last_state == NO_ACTION) //gimbal.last_state在最末尾赋值
					&&(gimbal.state == NO_ACTION)
					&&(HAL_GetTick() - no_action_time > debug_time))//yaw轴遥控器无输入，且等待延时(这里的延时同时可以给小陀螺到正常模式的底盘归到校准零点)后，遥控器输入时
		{
			//上一次为yaw轴遥控器无遥控器输入时，给定值为相对角（只给定一次），反馈值为相对角（反馈一直刷新），保持云台yaw轴在上一次位置不运动
			if(input_flag == 1)  //上一次是运动的,意味着下一次的静止位置需要改变，不能再为上一次的静止位置，因为运动了
					gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle; //让云台稳定
			//！！！↓这里有个HACK，上电后如果不运动，用相对角也能保持静止(没动遥控yaw_angle是不会有值的)，因为归中会归中到相对角零点(但是如果动小陀螺会有bug。。。)
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;//gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;用陀螺仪稳定也行，但是可能会温漂

			gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//记录yaw轴陀螺仪位置，刷新零轴
			
			input_flag = 0;//标志位置0

		}
		else if(gimbal.last_state == IS_ACTION && gimbal.state == IS_ACTION)//yaw轴遥控器有遥控器输入时
		{
			input_flag = 1;//yaw轴遥控器有遥控器输入时，标志位置1
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;//以（陀螺仪角度-记录角度的偏移量）为反馈值
			gimbal.pid.yaw_angle_ref += angle_yaw;//给定值为遥控器数据
		}
		//无运动趋势+有运动趋势但如动！
		else //这里是第一次进入模式总会走的路线，用于固定云台，遥控器无输入，且未达到延时时
		{    //(gimbal.last_state == NO_ACTION && gimbal.state == IS_ACTION ||gimbal.last_state == NO_ACTION && gimbal.state == NO_ACTION)
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;
			gimbal.pid.yaw_angle_ref = 0; //给定值清零	！！！很重要

		}
	}
  
	//PIT轴
	gimbal.pid.pit_angle_ref += (-angle_pit); //给定为遥控器数据
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;//反馈为相对角
  /* 软件限制pitch轴角度 */
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

  gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;//自瞄模式用
  gimbal.last_state = remote_is_action();//获取上次输入的状态
}

//分离模式
static void separate_handler(void)
{
  if(last_gimbal_mode != GIMBAL_SEPARATE_MODE) 
  {
    	gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;
		gimbal.pid.yaw_angle_fdb = yaw_180_to_900();
		gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;//刚进入时给定为当前相对角度，保持云台不动
		i_temp = 0;//用于记录yaw轴转了多少圈，每次进入该模式时清零重新记录
  }
	else
	{
		gimbal.pid.yaw_angle_fdb = yaw_180_to_900(); //电机角度转化，反馈值为相对角度（该函数用于将相对角度的范围从 -180~189 扩大到 -900~900）
		gimbal.pid.yaw_angle_ref += angle_yaw; //给定值为遥控器数据
		/* 软件限制yaw轴角度 */
		VAL_LIMIT(gimbal.pid.yaw_angle_ref, -900, 900);
	}
	
	//PIT
	gimbal.pid.pit_angle_ref += (-angle_pit);
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  /* 软件限制pitch轴角度 */
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);	

  gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;//自瞄模式用
  gimbal.last_state = remote_is_action();//获取上次输入的状态
}

//小陀螺模式 底盘旋转起来的时候，yaw轴的电机也是会动的，这样才能保持云台不动
static void dodge_handler(void)
{
  if(last_gimbal_mode != GIMBAL_DODGE_MODE) 
  {
    gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//刚进入该模式，记录陀螺仪角度，刷新零轴
		no_action_time = HAL_GetTick();
  }
	else
	{
		gimbal.state = remote_is_action(); //判断yaw轴是否有输入
		if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)
		{
			gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//记录yaw轴陀螺仪位置
    		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;
			gimbal.pid.yaw_angle_ref = 0;//遥控器无输入时，给定值为0	
			no_action_time = HAL_GetTick();
		}
		else if((gimbal.last_state == NO_ACTION) 
					&&(gimbal.state == NO_ACTION)
					&&(HAL_GetTick() - no_action_time > debug_time))
		{
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;//反馈值为（陀螺仪角度-记录角度的偏移量）
			gimbal.pid.yaw_angle_ref = 0; //遥控器无输入时，给定值为0

		}
		else if(gimbal.last_state == IS_ACTION && gimbal.state == IS_ACTION)
		{
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;//这才叫相对角
			gimbal.pid.yaw_angle_ref += angle_yaw;//遥控器有输入时，给定值为遥控器数据
		}
		else//(gimbal.last_state == NO_ACTION && gimbal.state == IS_ACTION ||gimbal.last_state == NO_ACTION && gimbal.state == NO_ACTION)
		{
			gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;
			gimbal.pid.yaw_angle_ref = 0;//遥控器无输入时，给定值为0	
		}
	}
  
	/*pitch轴*/
	gimbal.pid.pit_angle_ref += (-angle_pit);
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
	/* 软件限制pitch轴角度 */
	VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

	gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;
	gimbal.last_state = remote_is_action();//获取上次输入的状态

}

/** @brief 	   自瞄模式
**	@attention point重复建系-->gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;！！！
**  @author
**/
LADRC_NUM Vision_Angle_Pit =
{ 
	.h=0.002,//定时时间及时间步长
	.r=30,//跟踪速度参数
};
LADRC_NUM Vision_Angle_Yaw =
{ 
	.h=0.002,//定时时间及时间步长
	.r=30,//跟踪速度参数
};
float yaw_ref,pit_ref;
static void track_aimor_handler(void)
{
	/* 记录丢失角度 */
 	static float lost_pit;
	static float lost_yaw;
	/*  期望角度 */
	float yaw_ctrl;
	float pit_ctrl;

	if(last_gimbal_mode != GIMBAL_TRACK_ARMOR) //当上一次不为自瞄模式
	{
		gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;//备份陀螺仪数据，以免退出自瞄时冲突，刷新零轴
	}
	
	gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;//自瞄模式有自己的yaw_offset
 	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_angle;//yaw轴用陀螺仪(模拟相对角)
	
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle; //pitch反馈为陀螺仪角

	/* TD跟踪微分处理 */
	LADRC_TD(&Vision_Angle_Pit,  gimbal.sensor.pit_gyro_angle + pc_recv_mesg.aim_pitch);
	LADRC_TD(&Vision_Angle_Yaw,  gimbal.sensor.yaw_gyro_angle - gimbal.yaw_angle + pc_recv_mesg.aim_yaw);
	
	/* 视觉有效处理 */
	if (pc_recv_mesg.mode_Union.info.visual_valid == 1) //视觉识别成功标志位
	{
		//yaw的给定与反馈的差值一直都是 pc_recv_mesg.aim_yaw，及算法给过来的数据（pitch只与偏差有关）
//		yaw_ctrl = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_angle + pc_recv_mesg.aim_yaw;
// 	  	pit_ctrl =  gimbal.sensor.pit_gyro_angle + pc_recv_mesg.aim_pitch;
		yaw_ctrl = Vision_Angle_Yaw.v1; //v1为角度，v2为导数
		pit_ctrl = Vision_Angle_Pit.v1;
		
	}	
	/*视觉无效处理*/
  	else if (pc_recv_mesg.mode_Union.info.visual_valid == 0)	//视觉无效时，算法会把该值置0	
	{
			lost_pit = gimbal.sensor.pit_gyro_angle;//丢失目标后pitch不动
			lost_yaw = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_angle;	//丢失目标后yaw不动(此时yaw期望为当前反馈量)	
		
			pit_ctrl = lost_pit;
		    yaw_ctrl = lost_yaw;
	}
	
  	gimbal.pid.pit_angle_ref = pit_ctrl;
	gimbal.pid.yaw_angle_ref = yaw_ctrl;

	/* 软件限制pitch轴角度 */
	VAL_LIMIT(gimbal.pid.pit_angle_ref, -23, 36);

	gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//备份陀螺仪数据，以免退出自瞄时冲突(这里的offset是正常模式使用的,与yaw_angle无关)

}
/*初始化前馈补偿*/
void initFeedforwardParam(FFC *vFFC,float a,float b)
{

	vFFC->a = a;
	vFFC->b = b;
	vFFC->lastRin = 0;
	vFFC->perrRin = 0;
	vFFC->rin = 0;
}

/*实现前馈控制器*/
float getFeedforwardControl(FFC* vFFC,float v)//yaw轴
{
	vFFC->rin = v;
	float result = vFFC->a * (vFFC->rin - vFFC->lastRin) + vFFC->b * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);
	vFFC->perrRin = vFFC->lastRin;
	vFFC->lastRin = vFFC->rin;
	return result;
}
/*EASY_FFT*/
void Easy_FFC_Init(Easy_FFC_t* easy_ffc ,float K)
{
	easy_ffc->Kc = K;
}
void Easy_FFC_Calc(Easy_FFC_t* easy_ffc ,float K ,float gimbal_expect_ref)
{
	easy_ffc->Outner_Expect[NOW] = gimbal_expect_ref;
	easy_ffc->out = K*(easy_ffc->Outner_Expect[NOW] - easy_ffc->Outner_Expect[LAST]);
	easy_ffc->Outner_Expect[LAST] = easy_ffc->Outner_Expect[NOW];
}
