/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "bsp_rs485.h"
#include "ld2_motor.h"
#include "app_chassis_motor_ctrl.h"
#include "bsp_vofa.h"
#include "bsp_rc.h"
#include "app_ld2rs_task.h"
#include "app_led_indicator.h"
#include "arm_math.h"
#include "bsp_can_robostride.h"
#include "app_steer_chassis.h"
#include "bsp_rs485_bus2.h"
#include "ds_rs_um_motor.h"
#include "app_wheel_task.h"
#include "app_serial_ctrl.h"
#include "bsp_dwt.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_Vofa_UpperDisplay(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t g_fdcan1_last_tx_error_counter = 0U;
volatile uint32_t g_fdcan1_last_rx_error_counter = 0U;
volatile uint32_t g_fdcan1_tx_error_counter_rise_count = 0U;
volatile uint32_t g_fdcan1_error_counter_read_fail_count = 0U;
volatile uint32_t g_fdcan1_error_counter_poll_count = 0U;
void bsp_fdcan1_error_counter_poll(void)
{
    FDCAN_ErrorCountersTypeDef err_counter;
    static uint32_t last_tx_error_counter = 0U;
		
		g_fdcan1_error_counter_poll_count++;
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &err_counter) != HAL_OK) {
			 g_fdcan1_error_counter_read_fail_count++;
        return;
    }

    g_fdcan1_last_tx_error_counter = err_counter.TxErrorCnt;
    g_fdcan1_last_rx_error_counter = err_counter.RxErrorCnt;

    if (err_counter.TxErrorCnt > last_tx_error_counter) {
        g_fdcan1_tx_error_counter_rise_count++;
    }

    last_tx_error_counter = err_counter.TxErrorCnt;
}
 volatile uint8_t GPIOA_PIN0_FLAG = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//  float32_t sine = arm_sin_f32(12);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_UART7_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_SPI6_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
	
  /* USER CODE BEGIN 2 */
	
	/** @brief BSP 层：1、添加内核级别定时器DWT时钟外设，该外设只要内核CPU在跳动，则不会停止计时;
			@brief				 2、其次，DWT时钟作为基于Cortex-M的核心时钟，不会被中断打断;
			@brief				 3、最后，DWT时钟基于CPU，其精度为1/480M;
	*/
	if (!BSP_DWT_Init()) 
	{
			Error_Handler();
	}


  /** @brief BSP 层：初始化 RS485 总线 (USART3, 硬件 DE) */
  bsp_rs485_init(&g_rs485_bus, &huart3);

  /** @brief 中间件层：初始化 LD2-RS 设备句柄 */
  ld2_motor_init(&g_ld2rs_dev_m1, &g_rs485_bus, APP_LD2_MOTOR_SLAVE_ID_M1, APP_LD2_MOTOR_TIMEOUT_MS);
  ld2_motor_init(&g_ld2rs_dev_m2, &g_rs485_bus, APP_LD2_MOTOR_SLAVE_ID_M2, APP_LD2_MOTOR_TIMEOUT_MS);

  /** @brief 等待驱动器就绪，延迟 200ms */
  HAL_Delay(APP_SYSTEM_START_DELAY_MS);

  /** @brief 应用层：初始化电机控制参数 M1/M2 */
  app_chassis_ld2rs_motor_ctrl_init(&g_ld2rs_motor_ctrl_m1, &g_ld2rs_dev_m1, APP_MOTOR_CTRL_REF_SPEED_RPM_M1, 1u);
  app_chassis_ld2rs_motor_ctrl_init(&g_ld2rs_motor_ctrl_m2, &g_ld2rs_dev_m2, APP_MOTOR_CTRL_REF_SPEED_RPM_M2, 2u);

  /** @brief 回读驱动器参数 (仅读取, 不写入) */
  app_chassis_ld2rs_motor_ctrl_param_read_back(&g_ld2rs_dev_m1);
  app_chassis_ld2rs_motor_ctrl_param_read_back(&g_ld2rs_dev_m2);

  /** @brief 写入 M1/M2 速度环 PI 参数 */
  app_chassis_ld2rs_motor_ctrl_pi_init(&g_ld2rs_dev_m1, &g_ld2rs_motor_param_m1);
  app_chassis_ld2rs_motor_ctrl_pi_init(&g_ld2rs_dev_m2, &g_ld2rs_motor_param_m2);

  /** @brief 启动遥控接收 (UART5 DMA+IDLE, SBUS 协议) */
  bsp_rc_init(&g_rc);

  /** @brief 初始化非阻塞电机控制状态机 */
  app_ld2rs_task_init();

  /** @brief 初始化 WS4810 LED 状态指示灯 */
  app_led_indicator_init();

  /** @brief 初始化 FDCAN1 过滤器和 RX FIFO0 中断 */
  bsp_can_robostride_init();


/* ==== 一次性修正 CAN ID 顺序 (全部 4 台已连, 借助临时 ID=5 破环) ==== */
/* 烧录后运行一次, 确认 VOFA 反馈 motor_id 正确后注释掉下面这整段。      */

/* ① FR: 1→5 (临时避让, 释放 ID=1) */
//bsp_can_robostride_set_motor_id_once(1, 5);
//HAL_Delay(10);

///* ② FL: 2→1 */
//bsp_can_robostride_set_motor_id_once(2, 1);
//HAL_Delay(10);

///* ③ RL: 3→2 */
//bsp_can_robostride_set_motor_id_once(3, 2);
//HAL_Delay(10);

///* ④ RR: 4→3 */
//bsp_can_robostride_set_motor_id_once(4, 3);
//HAL_Delay(10);

///* ⑤ FR: 5→4 (从临时 ID 回到最终 ID) */
//bsp_can_robostride_set_motor_id_once(5, 4);

//while (1) {}  /* 改完即停, 不进主循环 */

  /* ================================================================ */

#if 1
    /* ================================================================ */
    /* 基于 UART2 RS485 的四轮轮毂电机测试初始化
     * ----------------------------------------------------------------
     * 说明：
     *   - 复用 bsp_rs485.c 通用状态机，新增 bsp_rs485_bus2 绑定到 huart2。
     *   - 4 个轮毂电机 Modbus ID：01=FL, 02=RL, 03=RR, 04=FR。
     *   - 电机型号：DS/RS/UM 系列低压伺服，A 轴速度模式。
     *   - APP 层使用非阻塞 FSM 轮询：READ D0.000 → WRITE F0.3.018。
     *   - 与原有 app_steer_chassis_init() 互不冲突，可并存。
     *   - 当前被注释掉，待后续接入底盘速度控制后再启用。
     */
    bsp_rs485_bus2_init();

    ds_rs_um_motor_init(&g_ds_rs_um_wheel_fl, &g_rs485_bus2, 1u, APP_WHEEL_TIMEOUT_MS);
    ds_rs_um_motor_init(&g_ds_rs_um_wheel_rl, &g_rs485_bus2, 2u, APP_WHEEL_TIMEOUT_MS);
    ds_rs_um_motor_init(&g_ds_rs_um_wheel_rr, &g_rs485_bus2, 3u, APP_WHEEL_TIMEOUT_MS);
    ds_rs_um_motor_init(&g_ds_rs_um_wheel_fr, &g_rs485_bus2, 4u, APP_WHEEL_TIMEOUT_MS);

    HAL_Delay(200u);  /* 等待驱动器就绪 */

    /* 设置控制模式为速度模式 (F0.1.002 = 1) */
    (void)ds_rs_um_motor_set_speed_mode(&g_ds_rs_um_wheel_fl);
    (void)ds_rs_um_motor_set_speed_mode(&g_ds_rs_um_wheel_rl);
    (void)ds_rs_um_motor_set_speed_mode(&g_ds_rs_um_wheel_rr);
    (void)ds_rs_um_motor_set_speed_mode(&g_ds_rs_um_wheel_fr);

    /* 电机使能 (F0.1.000 = 1) */
    (void)ds_rs_um_motor_set_enable(&g_ds_rs_um_wheel_fl, DS_RS_UM_MOTOR_ENABLE_ON);
    (void)ds_rs_um_motor_set_enable(&g_ds_rs_um_wheel_rl, DS_RS_UM_MOTOR_ENABLE_ON);
    (void)ds_rs_um_motor_set_enable(&g_ds_rs_um_wheel_rr, DS_RS_UM_MOTOR_ENABLE_ON);
    (void)ds_rs_um_motor_set_enable(&g_ds_rs_um_wheel_fr, DS_RS_UM_MOTOR_ENABLE_ON);

    /* 初始化 APP 层非阻塞四轮轮毂控制任务 */
    app_wheel_task_init();
    /* ================================================================ */
#endif
	app_steer_chassis_init();
  /** @brief 初始化串口控制模块 (UART7, PE7/PE8, 115200 8N1) */
  app_serial_ctrl_init();

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    app_rc_channels_check_lost(&g_rc);
    app_rc_channels_set_emergency_flag(&g_rc);

    app_serial_ctrl_poll();

    /* Control source: SWD DOWN or RC lost → serial; otherwise → RC */
    if (app_serial_ctrl_is_active()) {
        serial_command_t cmd;
        if (app_serial_ctrl_get_command(&cmd)) {
            app_steer_chassis_run_direct(cmd.vx_mps, cmd.vy_mps, cmd.wz_rad_s);
        } else {
            app_steer_chassis_run_direct(0.0f, 0.0f, 0.0f);
        }
    } else {
        app_steer_chassis_rc_control(&g_rc_filter);
    }

		
		//bsp_fdcan1_error_counter_poll();
		
		/*@brief: "dwt" vs "HAL_Gettick()"*/
		static uint32_t dwt_tick = 0;
		dwt_tick = BSP_DWT_GetTickMs(); //keil查看
		static uint32_t hal_tick = 0;
		hal_tick = HAL_GetTick(); //keil查看
		

#if APP_VOFA_JUSTFLOAT_ENABLE
    App_Vofa_UpperDisplay();
    /** @brief VOFA+ JustFloat 上位机波形发送 */
#endif

    app_led_indicator_update();
    /** @brief WS4810 LED 系统状态灯刷新 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* VOFA+ JustFloat 转速通道数据 (MotorRuntime 状态机周期写入) */
vofa_motor_info_t g_vofa_speed = {0.0f};

static void App_Vofa_UpperDisplay(void)
{
  extern volatile uint8_t g_emergency_stop_flag;
    /** @brief VOFA+ JustFloat 按周期发送波形数据 */
  {
      static uint32_t s_u32LastVofaTick = 0u;
      uint32_t now_tick = HAL_GetTick();
     if ((now_tick - s_u32LastVofaTick) >= APP_VOFA_JUSTFLOAT_PERIOD_MS) {

          const steer_chassis_t *chassis = app_steer_chassis_get_state();
          int16_t wheel_target_speed_rpm[APP_WHEEL_COUNT] = {0};

          app_wheel_task_get_target_speed_rpm(wheel_target_speed_rpm);

          float vofa_channels[21] = {

              (float)g_emergency_stop_flag,                  /* CH01 */
              (float)g_rc.lost_flag,                         /* CH02 */
              (float)g_rc.sw_st[eRC_SW_A].curr,              /* CH03 */

              /* 舵向给定 (°) */
              (float)chassis->modules[APP_STEER_MODULE_FL].robstride_target_angle_deg,  /* CH04 */
              (float)chassis->modules[APP_STEER_MODULE_RL].robstride_target_angle_deg,  /* CH05 */
              (float)chassis->modules[APP_STEER_MODULE_RR].robstride_target_angle_deg,  /* CH06 */
              (float)chassis->modules[APP_STEER_MODULE_FR].robstride_target_angle_deg,   /* CH07 */
              /* 舵向反馈 (°) */
             (float)chassis->modules[APP_STEER_MODULE_FL].feedback.position_raw_deg,  /* CH08 */
              (float)chassis->modules[APP_STEER_MODULE_RL].feedback.position_raw_deg,  /* CH09 */
             (float)chassis->modules[APP_STEER_MODULE_RR].feedback.position_raw_deg,  /* CH10 */  
             (float) chassis->modules[APP_STEER_MODULE_FR].feedback.position_raw_deg,  /* CH11 */

              /* 轮毂反转标志 (0=正转, 1=反转) */
             (float) chassis->modules[APP_STEER_MODULE_FL].hub_reverse_flag,  /* CH12 */
            (float)  chassis->modules[APP_STEER_MODULE_RL].hub_reverse_flag,  /* CH13 */
            (float)  chassis->modules[APP_STEER_MODULE_RR].hub_reverse_flag,  /* CH14 */
           (float)   chassis->modules[APP_STEER_MODULE_FR].hub_reverse_flag,   /* CH15 */
              /* 轮毂目标转速给定 (rpm) */
           (float)   chassis->modules[APP_STEER_MODULE_FL].hub_target_speed_mps,  /* CH16 */
            (float)  chassis->modules[APP_STEER_MODULE_RL].hub_target_speed_mps,  /* CH17 */
          (float)    chassis->modules[APP_STEER_MODULE_RR].hub_target_speed_mps,  /* CH18 */
            (float)  chassis->modules[APP_STEER_MODULE_FR].hub_target_speed_mps,  /* CH19 */
							 (float)s_steerwheel_chassis_task_20ms_flag,
              (float)  GPIOA_PIN0_FLAG

          };
          bsp_vofa_just_float(vofa_channels, 21);
          s_u32LastVofaTick = now_tick;
      }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
