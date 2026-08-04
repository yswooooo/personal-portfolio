#include "imu_task.h"
#include "gimbal_task.h"
#include "bsp_imu.h"
#include "pid.h"
#include "sys_config.h"
#include "math.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
#include "comm_task.h"
#include "bmi088driver.h"
#include "delay.h"
#include "MahonyAHRS.h"
#include "filters.h"
#include "bsp_dwt.h"
#include "dma.h"
#include "detect_task.h"
#include "ahrs.h"

#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

UBaseType_t imu_stack_surplus;
extern TaskHandle_t imu_Task_Handle;


volatile uint8_t imu_start_flag = 0;

extern IMU_Data_t BMI088;
imu_attitude_t atti;

float EulerAngle[3]={0};

float totalangle_transfer(float angle);

float imu_pid[3] = {200,0.1,0};


fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

//ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

//static uint8_t first_temperate;
//static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.??? ?? rad

//imu控温函数
void imu_temp_keep(void)
{
  PID_Struct_Init(&pid_imu_tmp, imu_pid[0],imu_pid[1],imu_pid[2],5000, 1000, INIT );
	if(BMI088.Temperature>45)
	{
		pid_calc(&pid_imu_tmp, BMI088.Temperature, 45);
		TIM_SetCompare1(TIM10,pid_imu_tmp.out);
	}
	else
	{
		TIM_SetCompare1(TIM10,5000-1); 
  }
}

/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], IMU_Data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->Gyro[0] * gyro_scale_factor[i][0] + bmi088->Gyro[1] * gyro_scale_factor[i][1] + bmi088->Gyro[2] * gyro_scale_factor[i][2] - bmi088->GyroOffset[i];
        accel[i] = bmi088->Accel[0] * accel_scale_factor[i][0] + bmi088->Accel[1] * accel_scale_factor[i][1] + bmi088->Accel[2] * accel_scale_factor[i][2] + accel_offset[i];
//        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}


void imu_task(void const *argu)
{
  
	uint32_t imu_wake_time = osKernelSysTick();//获取当前系统时钟 tick 数，并将其存储在 imu_wake_time 变量中

	imu_Task_Handle = xTaskGetHandle(pcTaskGetName(NULL));//获取当前任务的句柄并将其存储在 imu_Task_Handle 变量中。这在 FreeRTOS 中是一种常见的操作，通过任务句柄可以对任务进行操作或者监控任务的状态等。
	imu_start_flag = 1;

	AHRS_init(INS_quat, INS_accel, INS_mag);//根据加速度的数据，磁力计的数据进行四元数初始化
	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
	accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
	accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
	
  while(1)
  {
		//imu_temp_keep();//IMU恒温处理
    	BMI088_read(BMI088.Gyro, BMI088.Accel, &BMI088.Temperature);
		//rotate and zero drift 
		imu_cali_slove(INS_gyro, INS_accel, INS_mag, &BMI088);//旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式(未涉及magnetometer)
		//accel low-pass filter 低通滤波处理
		accel_fliter_1[0] = accel_fliter_2[0];
		accel_fliter_2[0] = accel_fliter_3[0];

		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

		accel_fliter_1[1] = accel_fliter_2[1];
		accel_fliter_2[1] = accel_fliter_3[1];

		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

		accel_fliter_1[2] = accel_fliter_2[2];
		accel_fliter_2[2] = accel_fliter_3[2];

		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


		AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);//根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
		get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);//根据四元数大小计算对应的欧拉角yaw，pitch，roll
		
		//弧度 转换到 角度
		INS_angle[0]=INS_angle[0]*RAD_TO_ANGLE;
		INS_angle[1]=INS_angle[1]*RAD_TO_ANGLE;
    	INS_angle[2]=INS_angle[2]*RAD_TO_ANGLE;
		
		//云台控制所需欧拉角、加速度
		gimbal.sensor.yaw_gyro_angle = totalangle_transfer(INS_angle[0]);
		gimbal.sensor.yaw_palstance  = BMI088.Gyro[2]*100;

		gimbal.sensor.pit_gyro_angle = INS_angle[1];
		gimbal.sensor.pit_palstance  = -BMI088.Gyro[0]*100;

		err_detector_hook(IMU_OFFLINE); //detect 检测是否掉线

		imu_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&imu_wake_time, IMU_TASK_PERIOD); //任务以绝对延时运行，时间为 1ms
  }

}

/* Transform into continuous values */
float totalangle_transfer(float angle)
{
  if (angle - atti.last_yaw > 330)
    atti.yaw_cnt--;
  else if (angle - atti.last_yaw < -330)
    atti.yaw_cnt++;
  
  atti.last_yaw = angle;
  
  atti.yaw   = angle + atti.yaw_cnt*360;
	return atti.yaw;
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)==1)
	{
		if(imu_start_flag)
		{
			if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
			{
				static BaseType_t xHigherPriorityTaskWoken;
				vTaskNotifyGiveFromISR(imu_Task_Handle,&xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		}
	}

	EXTI_ClearITPendingBit(EXTI_Line4); 
}


